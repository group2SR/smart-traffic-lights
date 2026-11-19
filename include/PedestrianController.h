#ifndef PEDESTRIAN_CONTROLLER_H
#define PEDESTRIAN_CONTROLLER_H

#include <stdint.h>
#include "TrafficConfig.h"
#include "FaultManager.h"

// ============================================================================
// PEDESTRIAN CONTROLLER MODULE (2 Independent Pushbuttons & 2 RG Signal Groups)
// ============================================================================
// Locked Hardware Mapping:
//   - Pedestrian 1 : Walk = GPIO 19, Don't Walk = GPIO 18, Button 1 = GPIO 32
//   - Pedestrian 2 : Walk = GPIO 16, Don't Walk = GPIO 17, Button 2 = GPIO 4
//
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

enum class PedestrianPriorityLevel : uint8_t {
    NONE            = 0,
    ELEVATED        = 1,
    MAX_WAIT_URGENT = 2
};

struct PedestrianPrioritySummary {
    bool                    anyPending;
    bool                    ped1Pending;
    bool                    ped2Pending;
    uint32_t                ped1WaitMs;
    uint32_t                ped2WaitMs;
    uint32_t                maxWaitMs;
    uint8_t                 maxSkippedCycles;
    bool                    urgencyActive;
    bool                    canServeTogether;
    PedestrianPriorityLevel level;
};

struct PedChannelState {
    uint8_t         buttonGpioPin;
    uint8_t         walkGpioPin;
    uint8_t         dontWalkGpioPin;
    bool            rawPinHigh;
    bool            rawPressed;
    bool            stablePressed;
    bool            awaitRelease;
    bool            stuckFault;
    bool            requestLatched;
    bool            requestFromSim;
    bool            servingWalk;
    bool            cycleObserved;
    uint8_t         skippedCycles;
    uint16_t        pressCount;
    uint16_t        releaseCount;
    uint32_t        lastChangeMs;
    uint32_t        pressStartMs;
    uint32_t        requestLatchedMs;
    PedSignalAspect aspect;
};

class PedestrianController {
public:
    PedestrianController() : inClearanceInterval_(false) { reset(0); }

    void reset(uint32_t nowMs) {
        for (uint8_t i = 0; i < TrafficConfig::PED_BUTTON_COUNT; ++i) {
            channels_[i].buttonGpioPin    = TrafficConfig::PED_BUTTON_PINS[i];
            channels_[i].walkGpioPin      = (i == 0) ? TrafficConfig::PIN_PED1_WALK : TrafficConfig::PIN_PED2_WALK;
            channels_[i].dontWalkGpioPin  = (i == 0) ? TrafficConfig::PIN_PED1_DONT_WALK : TrafficConfig::PIN_PED2_DONT_WALK;
            channels_[i].rawPinHigh       = true;
            channels_[i].rawPressed       = false;
            channels_[i].stablePressed    = false;
            channels_[i].awaitRelease     = false;
            channels_[i].stuckFault       = false;
            channels_[i].requestLatched   = false;
            channels_[i].requestFromSim   = false;
            channels_[i].servingWalk      = false;
            channels_[i].cycleObserved    = false;
            channels_[i].skippedCycles    = 0;
            channels_[i].pressCount       = 0;
            channels_[i].releaseCount     = 0;
            channels_[i].lastChangeMs     = nowMs;
            channels_[i].pressStartMs     = nowMs;
            channels_[i].requestLatchedMs = nowMs;
            channels_[i].aspect           = PedSignalAspect::SAFE_OFF;
        }
        inClearanceInterval_ = false;
    }

    void updateRawPinSampleOnly(uint8_t idx, bool rawPinHigh) {
        if (idx >= TrafficConfig::PED_BUTTON_COUNT) return;
        channels_[idx].rawPinHigh = rawPinHigh;
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
        ch.rawPinHigh = rawPinHigh;

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
                    if (ch.pressCount < 0xFFFFU) {
                        ch.pressCount++;
                    }
                    if (!ch.awaitRelease && !ch.stuckFault) {
                        ch.awaitRelease = true;
                        if (!ch.requestLatched) {
                            ch.requestLatched   = true;
                            ch.requestFromSim   = false;
                            ch.requestLatchedMs = nowMs;
                            ch.skippedCycles    = 0;
                            newlyLatched        = true;
                        } else if (alreadyPendingOut) {
                            *alreadyPendingOut = true;
                        }
                    }
                } else {
                    if (ch.releaseCount < 0xFFFFU) {
                        ch.releaseCount++;
                    }
                    ch.cycleObserved = true;
                    ch.awaitRelease  = false;
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
            channels_[i].rawPinHigh    = true;
            channels_[i].rawPressed    = false;
            channels_[i].stablePressed = false;
            channels_[i].lastChangeMs  = nowMs;
        }
    }

    bool requestPedestrian(uint8_t idx, bool fromSim, uint32_t nowMs = 0) {
        if (idx >= TrafficConfig::PED_BUTTON_COUNT) return false;
        if (!channels_[idx].requestLatched) {
            channels_[idx].requestLatched   = true;
            channels_[idx].requestFromSim   = fromSim;
            channels_[idx].requestLatchedMs = nowMs;
            channels_[idx].skippedCycles    = 0;
            return true;
        }
        return false;
    }

    void notifyVehiclePhaseServed() {
        for (uint8_t i = 0; i < TrafficConfig::PED_BUTTON_COUNT; ++i) {
            if (channels_[i].requestLatched && channels_[i].skippedCycles < 255) {
                channels_[i].skippedCycles++;
            }
        }
    }

    uint32_t getMaxWaitingTimeMs(uint32_t nowMs) const {
        uint32_t maxWait = 0;
        for (uint8_t i = 0; i < TrafficConfig::PED_BUTTON_COUNT; ++i) {
            if (channels_[i].requestLatched) {
                const uint32_t w = nowMs - channels_[i].requestLatchedMs;
                if (w > maxWait) {
                    maxWait = w;
                }
            }
        }
        return maxWait;
    }

    uint8_t getMaxSkippedCycles() const {
        uint8_t maxSkipped = 0;
        for (uint8_t i = 0; i < TrafficConfig::PED_BUTTON_COUNT; ++i) {
            if (channels_[i].requestLatched && channels_[i].skippedCycles > maxSkipped) {
                maxSkipped = channels_[i].skippedCycles;
            }
        }
        return maxSkipped;
    }

    PedestrianPrioritySummary evaluatePriority(uint32_t nowMs,
                                               const TrafficConfig::ConflictMatrixConfig& cm,
                                               uint32_t pedUrgencyWaitMs = TrafficConfig::DEFAULT_PED_URGENCY_WAIT_MS) const {
        PedestrianPrioritySummary s{};
        s.ped1Pending      = channels_[0].requestLatched;
        s.ped2Pending      = channels_[1].requestLatched;
        s.anyPending       = s.ped1Pending || s.ped2Pending;
        s.ped1WaitMs       = s.ped1Pending ? (nowMs - channels_[0].requestLatchedMs) : 0;
        s.ped2WaitMs       = s.ped2Pending ? (nowMs - channels_[1].requestLatchedMs) : 0;
        s.maxWaitMs        = (s.ped1WaitMs > s.ped2WaitMs) ? s.ped1WaitMs : s.ped2WaitMs;
        s.maxSkippedCycles = getMaxSkippedCycles();
        s.canServeTogether = (!cm.conflicts[3][4] && !cm.conflicts[4][3]);
        s.urgencyActive    = s.anyPending && (s.maxWaitMs >= pedUrgencyWaitMs || s.maxSkippedCycles >= 1);
        if (!s.anyPending) {
            s.level = PedestrianPriorityLevel::NONE;
        } else if (s.urgencyActive) {
            s.level = PedestrianPriorityLevel::MAX_WAIT_URGENT;
        } else {
            s.level = PedestrianPriorityLevel::ELEVATED;
        }
        return s;
    }

    static const char* priorityLevelToString(PedestrianPriorityLevel lvl) {
        switch (lvl) {
            case PedestrianPriorityLevel::MAX_WAIT_URGENT: return "URGENT";
            case PedestrianPriorityLevel::ELEVATED:        return "ELEVATED";
            case PedestrianPriorityLevel::NONE:
            default:                                       return "NONE";
        }
    }

    void beginWalkPhase(const TrafficConfig::ConflictMatrixConfig& cm = TrafficConfig::DEFAULT_CONFLICT_MATRIX) {
        inClearanceInterval_ = false;
        const bool p1 = channels_[0].requestLatched;
        const bool p2 = channels_[1].requestLatched;
        const bool compatible = (!cm.conflicts[3][4] && !cm.conflicts[4][3]);

        if (p1 && p2 && !compatible) {
            // Serve the older / higher-skipped crossing independently; keep the other latched
            const bool serveP1 = (channels_[0].skippedCycles > channels_[1].skippedCycles) ||
                                 (channels_[0].skippedCycles == channels_[1].skippedCycles &&
                                  channels_[0].requestLatchedMs <= channels_[1].requestLatchedMs);
            channels_[0].servingWalk = serveP1;
            channels_[0].aspect      = serveP1 ? PedSignalAspect::WALK : PedSignalAspect::DONT_WALK;
            channels_[1].servingWalk = !serveP1;
            channels_[1].aspect      = !serveP1 ? PedSignalAspect::WALK : PedSignalAspect::DONT_WALK;
            if (serveP1) channels_[0].skippedCycles = 0;
            else channels_[1].skippedCycles = 0;
            return;
        }

        const bool anySpecific = p1 || p2;
        for (uint8_t i = 0; i < TrafficConfig::PED_BUTTON_COUNT; ++i) {
            if (channels_[i].requestLatched || !anySpecific) {
                channels_[i].servingWalk   = true;
                channels_[i].skippedCycles = 0;
                channels_[i].aspect        = PedSignalAspect::WALK;
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
                channels_[i].skippedCycles  = 0;
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
