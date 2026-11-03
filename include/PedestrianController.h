#ifndef PEDESTRIAN_CONTROLLER_H
#define PEDESTRIAN_CONTROLLER_H

#include <stdint.h>
#include "TrafficConfig.h"
#include "FaultManager.h"

// ============================================================================
// PEDESTRIAN CONTROLLER MODULE (2 Independent Pushbuttons & 2 RG Signal Groups)
// ============================================================================
// Responsibilities:
// - Support two independent pedestrian request buttons (PED1 on GPIO 32, PED2 on GPIO 4).
// - Debounce button inputs (configurable pedDebounceMs, default 50 ms) with
//   hold-until-release suppression and stuck-button supervision (> maxInputStuckMs).
// - Queue requests received during vehicle phases and preserve them until served
//   at safe phase boundaries or explicitly reset/cleared.
// - Track WALK, CLEARANCE, and DONT_WALK states per pedestrian group.
// - Guarantee pedestrian RED (DONT_WALK) and GREEN (WALK) are NEVER active simultaneously.

enum class PedSignalAspect : uint8_t {
    SAFE_OFF  = 0, // Used during STARTUP_SAFE and FAULT_SAFE
    DONT_WALK = 1, // Pedestrian RED on, GREEN off
    WALK      = 2, // Pedestrian GREEN on, RED off
    CLEARANCE = 3  // Post-walk clearance: Pedestrian RED on, GREEN off, all vehicles held RED
};

struct PedChannelState {
    bool            rawPressed;
    bool            stablePressed;
    bool            awaitRelease;
    bool            stuckFault;
    bool            requestLatched;
    bool            requestFromSim;
    bool            servingWalk;
    uint32_t        lastChangeMs;
    uint32_t        pressStartMs;
    PedSignalAspect aspect;
};

class PedestrianController {
public:
    PedestrianController() { reset(0); }

    void reset(uint32_t nowMs) {
        for (uint8_t i = 0; i < TrafficConfig::PED_BUTTON_COUNT; ++i) {
            channels_[i].rawPressed     = false;
            channels_[i].stablePressed  = false;
            channels_[i].awaitRelease   = false;
            channels_[i].stuckFault     = false;
            channels_[i].requestLatched = false;
            channels_[i].requestFromSim = false;
            channels_[i].servingWalk    = false;
            channels_[i].lastChangeMs   = nowMs;
            channels_[i].pressStartMs   = nowMs;
            channels_[i].aspect         = PedSignalAspect::SAFE_OFF;
        }
        inClearanceInterval_ = false;
    }

    // Process physical button input (Active LOW when PED_BTN_ACTIVE_LOW == true)
    // Returns true if a new pedestrian request was latched on this tick.
    bool updateButtonChannel(uint8_t idx,
                             uint32_t nowMs,
                             bool rawPinHigh,
                             uint32_t debounceMs,
                             uint32_t maxStuckMs,
                             FaultManager& faultMgr,
                             bool* alreadyPendingOut = nullptr) {
        if (alreadyPendingOut) *alreadyPendingOut = false;
        if (idx >= TrafficConfig::PED_BUTTON_COUNT) return false;
        PedChannelState& ch = channels_[idx];

        const bool pressed = TrafficConfig::PED_BTN_ACTIVE_LOW ? !rawPinHigh : rawPinHigh;
        if (pressed != ch.rawPressed) {
            ch.rawPressed   = pressed;
            ch.lastChangeMs = nowMs;
        }

        bool newlyLatched = false;
        if ((nowMs - ch.lastChangeMs) >= debounceMs) {
            if (pressed != ch.stablePressed) {
                ch.stablePressed = pressed;
                if (ch.stablePressed) {
                    ch.pressStartMs = nowMs;
                    if (!ch.awaitRelease && !ch.stuckFault) {
                        ch.awaitRelease = true;
                        if (!ch.requestLatched) {
                            ch.requestLatched = true;
                            ch.requestFromSim = false;
                            newlyLatched      = true;
                        } else if (alreadyPendingOut) {
                            *alreadyPendingOut = true;
                        }
                    }
                } else {
                    ch.awaitRelease = false;
                    if (ch.stuckFault) {
                        ch.stuckFault = false;
                        faultMgr.setPedButtonFaultBit(idx, false);
                    }
                }
            } else if (ch.stablePressed && !ch.stuckFault) {
                if ((nowMs - ch.pressStartMs) >= maxStuckMs) {
                    ch.stuckFault = true;
                    faultMgr.setPedButtonFaultBit(idx, true);
                    faultMgr.recordNonFatalFault(nowMs, FaultCode::PED_BUTTON_STUCK_ACTIVE,
                                                 "Pedestrian pushbutton held continuously > maxInputStuckMs");
                }
            }
        }
        return newlyLatched;
    }

    void clearInvalidAdapterState(uint32_t nowMs) {
        for (uint8_t i = 0; i < TrafficConfig::PED_BUTTON_COUNT; ++i) {
            channels_[i].rawPressed    = false;
            channels_[i].stablePressed = false;
            channels_[i].lastChangeMs  = nowMs;
        }
    }

    bool requestPedestrian(uint8_t idx, bool fromSim) {
        if (idx >= TrafficConfig::PED_BUTTON_COUNT) return false;
        if (!channels_[idx].requestLatched) {
            channels_[idx].requestLatched = true;
            channels_[idx].requestFromSim = fromSim;
            return true;
        }
        return false;
    }

    void beginWalkPhase() {
        inClearanceInterval_ = false;
        const bool anySpecific = channels_[0].requestLatched || channels_[1].requestLatched;
        for (uint8_t i = 0; i < TrafficConfig::PED_BUTTON_COUNT; ++i) {
            if (channels_[i].requestLatched || !anySpecific) {
                channels_[i].servingWalk = true;
                channels_[i].aspect      = PedSignalAspect::WALK;
            } else {
                channels_[i].servingWalk = false;
                channels_[i].aspect      = PedSignalAspect::DONT_WALK;
            }
        }
        // Ensure Group 1 walks whenever PED_WALK state is active and Group 1 requested (or default)
        if (!channels_[0].servingWalk && !channels_[1].servingWalk) {
            channels_[0].servingWalk = true;
            channels_[0].aspect      = PedSignalAspect::WALK;
        }
    }

    void beginClearancePhase() {
        inClearanceInterval_ = true;
        for (uint8_t i = 0; i < TrafficConfig::PED_BUTTON_COUNT; ++i) {
            if (channels_[i].servingWalk) {
                channels_[i].requestLatched = false;
                channels_[i].requestFromSim = false;
                channels_[i].servingWalk    = false;
                channels_[i].aspect         = PedSignalAspect::CLEARANCE;
            } else {
                channels_[i].aspect         = PedSignalAspect::DONT_WALK;
            }
        }
    }

    void endClearancePhase() {
        inClearanceInterval_ = false;
        for (uint8_t i = 0; i < TrafficConfig::PED_BUTTON_COUNT; ++i) {
            channels_[i].servingWalk = false;
            channels_[i].aspect      = PedSignalAspect::DONT_WALK;
        }
    }

    void setAllDontWalk() {
        inClearanceInterval_ = false;
        for (uint8_t i = 0; i < TrafficConfig::PED_BUTTON_COUNT; ++i) {
            channels_[i].servingWalk = false;
            channels_[i].aspect      = PedSignalAspect::DONT_WALK;
        }
    }

    void setAllSafeOff() {
        inClearanceInterval_ = false;
        for (uint8_t i = 0; i < TrafficConfig::PED_BUTTON_COUNT; ++i) {
            channels_[i].servingWalk = false;
            channels_[i].aspect      = PedSignalAspect::SAFE_OFF;
        }
    }

    bool clearSimulatedRequests() {
        bool clearedAny = false;
        for (uint8_t i = 0; i < TrafficConfig::PED_BUTTON_COUNT; ++i) {
            if (channels_[i].requestLatched && channels_[i].requestFromSim) {
                channels_[i].requestLatched = false;
                channels_[i].requestFromSim = false;
                clearedAny = true;
            }
        }
        return clearedAny;
    }

    bool hasAnyDemand() const {
        return channels_[0].requestLatched || channels_[1].requestLatched;
    }

    bool hasDemand(uint8_t idx) const {
        if (idx >= TrafficConfig::PED_BUTTON_COUNT) return false;
        return channels_[idx].requestLatched;
    }

    bool isWalkActive(uint8_t idx) const {
        if (idx >= TrafficConfig::PED_BUTTON_COUNT) return false;
        return channels_[idx].aspect == PedSignalAspect::WALK;
    }

    bool isDontWalkActive(uint8_t idx) const {
        if (idx >= TrafficConfig::PED_BUTTON_COUNT) return false;
        return (channels_[idx].aspect == PedSignalAspect::DONT_WALK ||
                channels_[idx].aspect == PedSignalAspect::CLEARANCE);
    }

    bool isInClearance() const { return inClearanceInterval_; }

    const PedChannelState& getChannel(uint8_t idx) const {
        return channels_[idx < TrafficConfig::PED_BUTTON_COUNT ? idx : 0];
    }

private:
    PedChannelState channels_[TrafficConfig::PED_BUTTON_COUNT];
    bool            inClearanceInterval_;
};

#endif // PEDESTRIAN_CONTROLLER_H

