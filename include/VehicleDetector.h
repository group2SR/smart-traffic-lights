#ifndef VEHICLE_DETECTOR_H
#define VEHICLE_DETECTOR_H

#include <stdint.h>
#include "TrafficConfig.h"
#include "FaultManager.h"

// ============================================================================
// VEHICLE DETECTOR MODULE (6 Independent IR Presence Sensors)
// ============================================================================
// Responsibilities:
// - Read all 6 IR sensors independently (IR1..IR6).
// - Filter noisy sensor readings using configurable stability/debounce window.
// - Detect vehicle presence and absence per sensor and per assigned movement.
// - Detect stuck-active sensors (> maxInputStuckMs) and floating/chattering
//   inputs (> IR_MAX_TRANSITIONS_PER_WINDOW within 1 s) and isolate faulty
//   channels so a bad sensor never causes an unsafe green or monopolizes a phase.
// - NOTE: IR obstacle sensors are strictly vehicle-presence detectors. They are
//   NEVER interpreted as speed detectors, vehicle counters, or emergency detectors.

struct IrChannelStatus {
    bool     rawDetected;
    bool     stableDetected;
    bool     latchedDemand;
    bool     stuckFault;
    bool     chatterFault;
    bool     validated;
    uint8_t  assignedMovement; // MovementId (0=MAIN, 1=SIDE, 2=TURN)
    uint32_t lastChangeMs;
    uint32_t activeStartMs;
    uint32_t chatterWindowStartMs;
    uint8_t  transitionCountInWindow;
};

class VehicleDetector {
public:
    VehicleDetector() {
        applySensorConfig(TrafficConfig::DEFAULT_SENSOR_CONFIG);
        reset(0);
    }

    void reset(uint32_t nowMs) {
        for (uint8_t i = 0; i < TrafficConfig::IR_SENSOR_COUNT; ++i) {
            channels_[i].rawDetected             = false;
            channels_[i].stableDetected          = false;
            channels_[i].latchedDemand           = false;
            channels_[i].stuckFault              = false;
            channels_[i].chatterFault            = false;
            channels_[i].lastChangeMs            = nowMs;
            channels_[i].activeStartMs           = nowMs;
            channels_[i].chatterWindowStartMs    = nowMs;
            channels_[i].transitionCountInWindow = 0;
        }
    }

    bool applySensorConfig(const TrafficConfig::SensorConfig& cfg) {
        bool valid = true;
        config_ = cfg;
        if (config_.irStabilityMs < 10 || config_.irStabilityMs > 5000) {
            config_.irStabilityMs = TrafficConfig::IR_STABILITY_MS;
            valid = false;
        }
        if (config_.pedDebounceMs < 10 || config_.pedDebounceMs > 2000) {
            config_.pedDebounceMs = TrafficConfig::PED_DEBOUNCE_MS;
            valid = false;
        }
        if (config_.maxInputStuckMs < 1000 || config_.maxInputStuckMs > 600000) {
            config_.maxInputStuckMs = TrafficConfig::MAX_INPUT_STUCK_MS;
            valid = false;
        }
        for (uint8_t i = 0; i < TrafficConfig::IR_SENSOR_COUNT; ++i) {
            if (config_.sensorToMovement[i] >= TrafficConfig::VEHICLE_GROUP_COUNT) {
                config_.sensorToMovement[i] = TrafficConfig::DEFAULT_SENSOR_CONFIG.sensorToMovement[i];
                valid = false;
            }
            channels_[i].assignedMovement = config_.sensorToMovement[i];
            channels_[i].validated = ((config_.sensorValidatedMask & (1U << i)) != 0);
        }
        return valid;
    }

    const TrafficConfig::SensorConfig& getSensorConfig() const { return config_; }

    void setAllValidated(bool validated) {
        config_.sensorValidatedMask = validated ? 0x3F : 0x00;
        for (uint8_t i = 0; i < TrafficConfig::IR_SENSOR_COUNT; ++i) {
            channels_[i].validated = validated;
            if (!validated) {
                channels_[i].rawDetected    = false;
                channels_[i].stableDetected = false;
                channels_[i].latchedDemand  = false;
            }
        }
    }

    void setChannelValidated(uint8_t idx, bool validated) {
        if (idx >= TrafficConfig::IR_SENSOR_COUNT) return;
        if (validated) {
            config_.sensorValidatedMask |= static_cast<uint8_t>(1U << idx);
        } else {
            config_.sensorValidatedMask &= static_cast<uint8_t>(~(1U << idx));
            channels_[idx].rawDetected    = false;
            channels_[idx].stableDetected = false;
            channels_[idx].latchedDemand  = false;
        }
        channels_[idx].validated = validated;
    }

    bool isChannelValidated(uint8_t idx) const {
        if (idx >= TrafficConfig::IR_SENSOR_COUNT) return false;
        return channels_[idx].validated;
    }

    // Update a single IR sensor channel from raw GPIO pin state (HIGH = true, LOW = false)
    // Returns true if a new vehicle arrival was latched on this tick.
    bool updateChannel(uint8_t idx, uint32_t nowMs, bool rawPinHigh, FaultManager& faultMgr) {
        if (idx >= TrafficConfig::IR_SENSOR_COUNT) return false;
        IrChannelStatus& ch = channels_[idx];

        if (!ch.validated) {
            ch.rawDetected    = false;
            ch.stableDetected = false;
            ch.latchedDemand  = false;
            return false;
        }

        const bool detected = TrafficConfig::IR_SNS_ACTIVE_LOW ? !rawPinHigh : rawPinHigh;

        // Chatter / disconnected floating input supervision
        if ((nowMs - ch.chatterWindowStartMs) >= TrafficConfig::IR_CHATTER_WINDOW_MS) {
            ch.chatterWindowStartMs    = nowMs;
            ch.transitionCountInWindow = 0;
        }

        if (detected != ch.rawDetected) {
            ch.rawDetected  = detected;
            ch.lastChangeMs = nowMs;
            if (ch.transitionCountInWindow < 255) {
                ch.transitionCountInWindow++;
            }
            if (ch.transitionCountInWindow > TrafficConfig::IR_MAX_TRANSITIONS_PER_WINDOW) {
                ch.chatterFault   = true;
                ch.stableDetected = false;
                ch.latchedDemand  = false;
                faultMgr.setSensorFaultBit(idx, true);
                faultMgr.recordNonFatalFault(nowMs, FaultCode::SENSOR_DISCONNECTED_CHATTER,
                                             "IR sensor excessive oscillation/chatter detected; suppressing channel");
                return false;
            }
        }

        bool newArrival = false;
        if ((nowMs - ch.lastChangeMs) >= config_.irStabilityMs) {
            // Stable reading achieved; clear chatter fault if it was previously set
            if (ch.chatterFault) {
                ch.chatterFault = false;
                if (!ch.stuckFault) {
                    faultMgr.setSensorFaultBit(idx, false);
                }
            }

            if (detected != ch.stableDetected) {
                ch.stableDetected = detected;
                if (ch.stableDetected) {
                    ch.activeStartMs = nowMs;
                    ch.stuckFault    = false;
                    faultMgr.setSensorFaultBit(idx, false);
                    if (!ch.latchedDemand) {
                        ch.latchedDemand = true;
                        newArrival       = true;
                    }
                } else {
                    if (ch.stuckFault) {
                        ch.stuckFault = false;
                        faultMgr.setSensorFaultBit(idx, false);
                    }
                }
            } else if (ch.stableDetected && !ch.stuckFault) {
                if ((nowMs - ch.activeStartMs) >= config_.maxInputStuckMs) {
                    ch.stuckFault    = true;
                    ch.latchedDemand = false;
                    faultMgr.setSensorFaultBit(idx, true);
                    faultMgr.recordNonFatalFault(nowMs, FaultCode::SENSOR_STUCK_ACTIVE,
                                                 "IR sensor active continuously > maxInputStuckMs; suppressing channel");
                }
            }
        }
        return newArrival;
    }

    void clearInvalidAdapterState(uint32_t nowMs) {
        for (uint8_t i = 0; i < TrafficConfig::IR_SENSOR_COUNT; ++i) {
            channels_[i].rawDetected    = false;
            channels_[i].stableDetected = false;
            channels_[i].lastChangeMs   = nowMs;
        }
    }

    void clearLatchedDemandForMovement(TrafficConfig::MovementId movement) {
        const uint8_t m = static_cast<uint8_t>(movement);
        for (uint8_t i = 0; i < TrafficConfig::IR_SENSOR_COUNT; ++i) {
            if (channels_[i].assignedMovement == m) {
                channels_[i].latchedDemand = false;
            }
        }
    }

    bool hasActivePresenceForMovement(TrafficConfig::MovementId movement) const {
        const uint8_t m = static_cast<uint8_t>(movement);
        for (uint8_t i = 0; i < TrafficConfig::IR_SENSOR_COUNT; ++i) {
            const IrChannelStatus& ch = channels_[i];
            if (ch.assignedMovement == m && ch.validated && !ch.stuckFault && !ch.chatterFault && ch.stableDetected) {
                return true;
            }
        }
        return false;
    }

    bool hasDemandForMovement(TrafficConfig::MovementId movement) const {
        const uint8_t m = static_cast<uint8_t>(movement);
        for (uint8_t i = 0; i < TrafficConfig::IR_SENSOR_COUNT; ++i) {
            const IrChannelStatus& ch = channels_[i];
            if (ch.assignedMovement == m && ch.validated && !ch.stuckFault && !ch.chatterFault &&
                (ch.latchedDemand || ch.stableDetected)) {
                return true;
            }
        }
        return false;
    }

    uint8_t countActiveSensorsForMovement(TrafficConfig::MovementId movement) const {
        const uint8_t m = static_cast<uint8_t>(movement);
        uint8_t count = 0;
        for (uint8_t i = 0; i < TrafficConfig::IR_SENSOR_COUNT; ++i) {
            const IrChannelStatus& ch = channels_[i];
            if (ch.assignedMovement == m && ch.validated && !ch.stuckFault && !ch.chatterFault &&
                (ch.stableDetected || ch.latchedDemand)) {
                count++;
            }
        }
        return count;
    }

    const IrChannelStatus& getChannel(uint8_t idx) const {
        return channels_[idx < TrafficConfig::IR_SENSOR_COUNT ? idx : 0];
    }

    bool isAnyChannelFaulted() const {
        for (uint8_t i = 0; i < TrafficConfig::IR_SENSOR_COUNT; ++i) {
            if (channels_[i].stuckFault || channels_[i].chatterFault) return true;
        }
        return false;
    }

private:
    TrafficConfig::SensorConfig config_;
    IrChannelStatus channels_[TrafficConfig::IR_SENSOR_COUNT];
};

#endif // VEHICLE_DETECTOR_H

