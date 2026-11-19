#ifndef VEHICLE_DETECTOR_H
#define VEHICLE_DETECTOR_H

#include <stdint.h>
#include "TrafficConfig.h"
#include "FaultManager.h"

// ============================================================================
// VEHICLE DETECTOR MODULE (6 Independent IR Presence Sensors — 2 Per Road)
// ============================================================================
// Locked Hardware Sensor-to-Road Architecture:
//   - Main Road      : Sensor 1 = GPIO 33 (Ch 0, INPUT_PULLUP)
//                      Sensor 2 = GPIO 34 (Ch 1, INPUT - input-only)
//   - Side Road      : Sensor 1 = GPIO 35 (Ch 2, INPUT - input-only)
//                      Sensor 2 = GPIO 36 (Ch 3, INPUT - input-only)
//   - Turn Movement  : Sensor 1 = GPIO 39 (Ch 4, INPUT - input-only)
//                      Sensor 2 = GPIO 15 (Ch 5, INPUT_PULLUP - MTDO strap)
//
// Responsibilities:
// - Read all 6 IR sensors independently (IR1..IR6), two per road (First & Second).
// - Maintain independent raw input state, debounced state, and latched demand per sensor.
// - Filter noisy sensor readings using configurable stability/debounce window (100 ms).
// - Detect vehicle presence and absence per sensor position and per assigned road.
// - Detect stuck-active sensors (> maxInputStuckMs) and floating/chattering
//   inputs (> IR_MAX_TRANSITIONS_PER_WINDOW within 1 s) and isolate faulty
//   channels so a bad sensor never causes an unsafe green or blinds the second
//   sensor on the same road.
// - NOTE: IR obstacle sensors are strictly vehicle-presence detectors. They are
//   NEVER interpreted as speed detectors, vehicle counters, or emergency detectors.

struct IrChannelStatus {
    uint8_t  gpioPin;              // Locked physical ESP32 GPIO pin (33, 34, 35, 36, 39, 15)
    uint8_t  assignedMovement;     // MovementId (0=MAIN_ROAD, 1=SIDE_ROAD, 2=PROTECTED_TURN)
    uint8_t  sensorPositionOnRoad; // 0 = First sensor (Sensor 1), 1 = Second sensor (Sensor 2)
    bool     rawPinHigh;           // Independent raw GPIO pin level (true = HIGH, false = LOW)
    bool     rawDetected;          // Instantaneous active-LOW obstacle state
    bool     stableDetected;       // Debounced obstacle presence state
    bool     latchedDemand;        // Latched vehicle demand awaiting phase service
    bool     stuckFault;           // True if continuously active > maxInputStuckMs
    bool     chatterFault;         // True if oscillating > IR_MAX_TRANSITIONS_PER_WINDOW / s
    bool     validated;            // True if physical sensor channel is commissioned
    bool     cycleObserved;        // True if a full clean detect + release cycle has been observed
    bool     rawShadowDetected;    // Internal electrical shadow state for pre-commissioning check
    uint16_t rawSampleCount;       // Total raw electrical samples collected (saturating)
    uint16_t detectionCount;       // Number of debounced vehicle arrivals detected
    uint16_t releaseCount;         // Number of debounced vehicle departures (releases) detected
    uint32_t lastChangeMs;
    uint32_t activeStartMs;
    uint32_t chatterWindowStartMs;
    uint8_t  transitionCountInWindow;
};

enum class RoadPassageState : uint8_t {
    IDLE                 = 0,
    ZONE_OCCUPIED        = 1,
    BOTH_OCCUPIED        = 2,
    INCOMPLETE_WAIT_CLEAR= 3,
    FAULT_SUSPENDED      = 4
};

struct RoadPassageStats {
    RoadPassageState state;
    bool             frontBlocked;
    bool             rearBlocked;
    bool             bothOccupied;
    bool             congestionSuspended;
    bool             rearWasBlockedInCycle;
    uint32_t         zoneStartMs;
    uint32_t         bothOccupiedStartMs;
    uint32_t         lastPassageMs;
    uint32_t         confirmedPassages;
    uint16_t         rollingPassages;
    uint16_t         incompleteSequences;
    uint16_t         arrivalRateTenthsPerMin; // e.g., 25 = 2.5 veh/min
    uint32_t         rollingWindowStartMs;
};

class VehicleDetector {
public:
    VehicleDetector() : config_(TrafficConfig::DEFAULT_SENSOR_CONFIG) {
        applySensorConfig(TrafficConfig::DEFAULT_SENSOR_CONFIG);
        reset(0);
    }

    void reset(uint32_t nowMs) {
        lastSampleMs_ = nowMs;
        for (uint8_t i = 0; i < TrafficConfig::IR_SENSOR_COUNT; ++i) {
            channels_[i].gpioPin                 = TrafficConfig::IR_SENSOR_PINS[i];
            channels_[i].rawPinHigh              = true;
            channels_[i].rawDetected             = false;
            channels_[i].stableDetected          = false;
            channels_[i].latchedDemand           = false;
            channels_[i].stuckFault              = false;
            channels_[i].chatterFault            = false;
            channels_[i].cycleObserved           = false;
            channels_[i].rawShadowDetected       = false;
            channels_[i].rawSampleCount          = 0;
            channels_[i].detectionCount          = 0;
            channels_[i].releaseCount            = 0;
            channels_[i].lastChangeMs            = nowMs;
            channels_[i].activeStartMs           = nowMs;
            channels_[i].chatterWindowStartMs    = nowMs;
            channels_[i].transitionCountInWindow = 0;
        }
        recomputeSensorPositionsOnRoads();
        for (uint8_t r = 0; r < TrafficConfig::VEHICLE_GROUP_COUNT; ++r) {
            roadPassages_[r] = RoadPassageStats{};
            roadPassages_[r].state = RoadPassageState::IDLE;
            roadPassages_[r].rollingWindowStartMs = nowMs;
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
            channels_[i].gpioPin          = TrafficConfig::IR_SENSOR_PINS[i];
            channels_[i].assignedMovement = config_.sensorToMovement[i];
            channels_[i].validated        = ((config_.sensorValidatedMask & (1U << i)) != 0);
        }
        recomputeSensorPositionsOnRoads();
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

    bool isAnyChannelValidated() const {
        return (config_.sensorValidatedMask & 0x3FU) != 0;
    }

    bool isAllChannelsValidated() const {
        return (config_.sensorValidatedMask & 0x3FU) == 0x3FU;
    }

    uint8_t getValidatedMask() const {
        return static_cast<uint8_t>(config_.sensorValidatedMask & 0x3FU);
    }

    // Electrical input validation check for commissioning a physical IR channel
    bool validateChannelElectricalHealth(uint8_t idx, const char** failureReason = nullptr) const {
        if (idx >= TrafficConfig::IR_SENSOR_COUNT) {
            if (failureReason) *failureReason = "Invalid IR sensor channel index";
            return false;
        }
        const IrChannelStatus& ch = channels_[idx];
        if (ch.rawSampleCount == 0) {
            if (failureReason) *failureReason = "IR sensor GPIO has not been sampled yet";
            return false;
        }
        if (ch.stuckFault) {
            if (failureReason) *failureReason = "IR sensor channel has active stuck-low fault";
            return false;
        }
        if (ch.chatterFault) {
            if (failureReason) *failureReason = "IR sensor channel has active chatter/oscillation fault";
            return false;
        }
        if (!ch.rawPinHigh) {
            if (failureReason) *failureReason = "IR sensor GPIO is held LOW (check 3.3V pull-up / sensor power / active obstacle)";
            return false;
        }
        if (!ch.cycleObserved) {
            if (failureReason) *failureReason = "IR sensor requires physical detect-and-release cycle (Cycle=YES); idle HIGH alone is insufficient";
            return false;
        }
        return true;
    }

    void updateRawPinSampleOnly(uint8_t idx, bool rawPinHigh) {
        if (idx >= TrafficConfig::IR_SENSOR_COUNT) return;
        IrChannelStatus& ch = channels_[idx];
        ch.rawPinHigh = rawPinHigh;
        if (ch.rawSampleCount < 0xFFFFU) {
            ch.rawSampleCount++;
        }
        const bool detected = TrafficConfig::IR_SNS_ACTIVE_LOW ? !rawPinHigh : rawPinHigh;
        if (detected && !ch.rawShadowDetected) {
            ch.rawShadowDetected = true;
        } else if (!detected && ch.rawShadowDetected) {
            ch.rawShadowDetected = false;
            ch.cycleObserved     = true;
        }
    }

    // Update a single IR sensor channel from raw GPIO pin state (HIGH = true, LOW = false)
    // Returns true if a new vehicle arrival was latched on this tick.
    bool updateChannel(uint8_t idx, uint32_t nowMs, bool rawPinHigh, FaultManager& faultMgr) {
        if (idx >= TrafficConfig::IR_SENSOR_COUNT) return false;
        lastSampleMs_ = nowMs;
        IrChannelStatus& ch = channels_[idx];
        ch.rawPinHigh = rawPinHigh;
        if (ch.rawSampleCount < 0xFFFFU) {
            ch.rawSampleCount++;
        }

        const bool detected = TrafficConfig::IR_SNS_ACTIVE_LOW ? !rawPinHigh : rawPinHigh;

        if (!ch.validated) {
            // Track pre-commissioning shadow cycle (detect -> release) without triggering demand
            if (detected && !ch.rawShadowDetected) {
                ch.rawShadowDetected = true;
            } else if (!detected && ch.rawShadowDetected) {
                ch.rawShadowDetected = false;
                ch.cycleObserved     = true;
            }
            ch.rawDetected    = false;
            ch.stableDetected = false;
            ch.latchedDemand  = false;
            return false;
        }

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
                    if (ch.detectionCount < 0xFFFFU) {
                        ch.detectionCount++;
                    }
                    if (!ch.latchedDemand) {
                        ch.latchedDemand = true;
                        newArrival       = true;
                    }
                } else {
                    if (ch.releaseCount < 0xFFFFU) {
                        ch.releaseCount++;
                    }
                    ch.cycleObserved = true;
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
            channels_[i].rawPinHigh     = true;
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

    // Look up sensor channel index (0..5) for a given road and sensor position (0 = Sensor 1, 1 = Sensor 2)
    int8_t getRoadSensorChannelIndex(TrafficConfig::MovementId road, uint8_t sensorPosition) const {
        const uint8_t r = static_cast<uint8_t>(road);
        if (r >= TrafficConfig::VEHICLE_GROUP_COUNT || sensorPosition >= TrafficConfig::SENSORS_PER_ROAD) {
            return -1;
        }
        uint8_t matchOrder = 0;
        for (uint8_t i = 0; i < TrafficConfig::IR_SENSOR_COUNT; ++i) {
            if (channels_[i].assignedMovement == r) {
                if (matchOrder == sensorPosition) {
                    return static_cast<int8_t>(i);
                }
                matchOrder++;
            }
        }
        return static_cast<int8_t>(TrafficConfig::ROAD_IR_SENSOR_CHANNELS[r][sensorPosition]);
    }

    const IrChannelStatus& getRoadSensor(TrafficConfig::MovementId road, uint8_t sensorPosition) const {
        const int8_t idx = getRoadSensorChannelIndex(road, sensorPosition);
        return channels_[(idx >= 0 && idx < TrafficConfig::IR_SENSOR_COUNT) ? idx : 0];
    }

    bool isRoadSensorDetected(TrafficConfig::MovementId road, uint8_t sensorPosition) const {
        const int8_t idx = getRoadSensorChannelIndex(road, sensorPosition);
        if (idx < 0) return false;
        const IrChannelStatus& ch = channels_[idx];
        return ch.validated && !ch.stuckFault && !ch.chatterFault && ch.stableDetected;
    }

    bool isRoadSensorLatched(TrafficConfig::MovementId road, uint8_t sensorPosition) const {
        const int8_t idx = getRoadSensorChannelIndex(road, sensorPosition);
        if (idx < 0) return false;
        const IrChannelStatus& ch = channels_[idx];
        return ch.validated && !ch.stuckFault && !ch.chatterFault && ch.latchedDemand;
    }

    bool isRoadSensorFaulted(TrafficConfig::MovementId road, uint8_t sensorPosition) const {
        const int8_t idx = getRoadSensorChannelIndex(road, sensorPosition);
        if (idx < 0) return false;
        const IrChannelStatus& ch = channels_[idx];
        return ch.stuckFault || ch.chatterFault;
    }

    int8_t findChannelByGpio(uint8_t gpioPin) const {
        for (uint8_t i = 0; i < TrafficConfig::IR_SENSOR_COUNT; ++i) {
            if (TrafficConfig::IR_SENSOR_PINS[i] == gpioPin) {
                return static_cast<int8_t>(i);
            }
        }
        return -1;
    }

    void updateRoadPassageTracking(uint32_t nowMs) {
        for (uint8_t r = 0; r < TrafficConfig::VEHICLE_GROUP_COUNT; ++r) {
            const TrafficConfig::MovementId road = static_cast<TrafficConfig::MovementId>(r);
            RoadPassageStats& ps = roadPassages_[r];

            if ((nowMs - ps.rollingWindowStartMs) >= TrafficConfig::PASSAGE_ROLLING_WINDOW_MS) {
                ps.rollingWindowStartMs    = nowMs;
                ps.rollingPassages         = 0;
                ps.arrivalRateTenthsPerMin = 0;
            }

            const int8_t fIdx = getRoadSensorChannelIndex(road, 0);
            const int8_t rIdx = getRoadSensorChannelIndex(road, 1);
            if (fIdx < 0 || rIdx < 0) continue;

            const IrChannelStatus& fCh = channels_[fIdx];
            const IrChannelStatus& rCh = channels_[rIdx];
            const bool sensorsHealthy = fCh.validated && rCh.validated &&
                                        !fCh.stuckFault && !fCh.chatterFault &&
                                        !rCh.stuckFault && !rCh.chatterFault;

            const bool fBlk = fCh.validated && !fCh.stuckFault && !fCh.chatterFault && fCh.stableDetected;
            const bool rBlk = rCh.validated && !rCh.stuckFault && !rCh.chatterFault && rCh.stableDetected;
            ps.frontBlocked = fBlk;
            ps.rearBlocked  = rBlk;
            ps.bothOccupied = (fBlk && rBlk);

            if (!sensorsHealthy) {
                if (ps.state != RoadPassageState::FAULT_SUSPENDED) {
                    ps.state                 = RoadPassageState::FAULT_SUSPENDED;
                    ps.congestionSuspended   = true;
                    ps.rearWasBlockedInCycle = false;
                }
                continue;
            }

            if (ps.state == RoadPassageState::FAULT_SUSPENDED) {
                if (!fBlk && !rBlk) {
                    ps.state               = RoadPassageState::IDLE;
                    ps.congestionSuspended = false;
                }
                continue;
            }

            // Reset tracking sequence cleanly whenever both sensors become clear
            if (!fBlk && !rBlk) {
                if (ps.state == RoadPassageState::ZONE_OCCUPIED && ps.rearWasBlockedInCycle) {
                    // Front cleared before rear completed; if rear also cleared on same tick, treat as incomplete
                    ps.incompleteSequences++;
                }
                ps.state                 = RoadPassageState::IDLE;
                ps.congestionSuspended   = false;
                ps.rearWasBlockedInCycle = false;
                continue;
            }

            switch (ps.state) {
                case RoadPassageState::IDLE:
                    if (fBlk && !rBlk) {
                        ps.state                 = RoadPassageState::ZONE_OCCUPIED;
                        ps.zoneStartMs           = nowMs;
                        ps.congestionSuspended   = false;
                        ps.rearWasBlockedInCycle = false;
                    } else if (fBlk && rBlk) {
                        // Both blocked simultaneously from idle -> cannot establish vehicle separation
                        ps.state                 = RoadPassageState::BOTH_OCCUPIED;
                        ps.bothOccupiedStartMs   = nowMs;
                        ps.congestionSuspended   = true;
                        ps.rearWasBlockedInCycle = false;
                    } else {
                        // Rear blocked without front -> invalid/incomplete sequence
                        ps.incompleteSequences++;
                        ps.state                 = RoadPassageState::INCOMPLETE_WAIT_CLEAR;
                        ps.rearWasBlockedInCycle = false;
                    }
                    break;

                case RoadPassageState::ZONE_OCCUPIED:
                    if (!fBlk && rBlk) {
                        // Front sensor cleared before rear activation/completion
                        ps.incompleteSequences++;
                        ps.state                 = RoadPassageState::INCOMPLETE_WAIT_CLEAR;
                        ps.rearWasBlockedInCycle = false;
                    } else if (fBlk && rBlk) {
                        if (!ps.rearWasBlockedInCycle) {
                            ps.rearWasBlockedInCycle = true;
                            ps.bothOccupiedStartMs   = nowMs;
                        } else if ((nowMs - ps.bothOccupiedStartMs) >= TrafficConfig::PASSAGE_BOTH_CONGESTION_MS) {
                            // Continuous simultaneous blockage -> congestion; suspend counting
                            ps.state                 = RoadPassageState::BOTH_OCCUPIED;
                            ps.congestionSuspended   = true;
                            ps.rearWasBlockedInCycle = false;
                        }
                    } else if (fBlk && !rBlk) {
                        if (ps.rearWasBlockedInCycle) {
                            // Valid rear blocked-to-clear cycle while front remained blocked -> +1 passage
                            ps.confirmedPassages++;
                            if (ps.rollingPassages < 0xFFFFU) {
                                ps.rollingPassages++;
                            }
                            ps.lastPassageMs         = nowMs;
                            ps.zoneStartMs           = nowMs; // Reset zone timeout for next consecutive vehicle
                            ps.rearWasBlockedInCycle = false;
                            const uint32_t winElapsed = nowMs - ps.rollingWindowStartMs;
                            const uint32_t effWinMs   = (winElapsed < 10000U) ? 10000U : winElapsed;
                            ps.arrivalRateTenthsPerMin = static_cast<uint16_t>((static_cast<uint32_t>(ps.rollingPassages) * 600000UL) / effWinMs);
                        } else if ((nowMs - ps.zoneStartMs) >= TrafficConfig::PASSAGE_SEQUENCE_TIMEOUT_MS) {
                            ps.incompleteSequences++;
                            ps.state                 = RoadPassageState::INCOMPLETE_WAIT_CLEAR;
                            ps.rearWasBlockedInCycle = false;
                        }
                    }
                    break;

                case RoadPassageState::BOTH_OCCUPIED:
                    if (fBlk && !rBlk) {
                        // Rear cleared after congestion; resume valid counting from clean zone state
                        ps.state                 = RoadPassageState::ZONE_OCCUPIED;
                        ps.zoneStartMs           = nowMs;
                        ps.congestionSuspended   = false;
                        ps.rearWasBlockedInCycle = false;
                    } else if (!fBlk && rBlk) {
                        ps.incompleteSequences++;
                        ps.state                 = RoadPassageState::INCOMPLETE_WAIT_CLEAR;
                        ps.congestionSuspended   = false;
                        ps.rearWasBlockedInCycle = false;
                    }
                    break;

                case RoadPassageState::INCOMPLETE_WAIT_CLEAR:
                default:
                    // Wait until both sensors become clear
                    break;
            }
        }
    }

    const RoadPassageStats& getRoadPassageStats(TrafficConfig::MovementId road) const {
        const uint8_t r = static_cast<uint8_t>(road);
        return roadPassages_[r < TrafficConfig::VEHICLE_GROUP_COUNT ? r : 0];
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

    bool isApproachFaulted(TrafficConfig::MovementId road) const {
        return isRoadSensorFaulted(road, 0) || isRoadSensorFaulted(road, 1);
    }

    bool isApproachDemand(TrafficConfig::MovementId road) const {
        return hasDemandForMovement(road);
    }

    bool isApproachPresence(TrafficConfig::MovementId road) const {
        return hasActivePresenceForMovement(road);
    }

    bool isAnySensorFaulted() const {
        return isAnyChannelFaulted();
    }

    bool isAnySensorStale(uint32_t nowMs, uint32_t staleTimeoutMs = 5000) const {
        if (nowMs < lastSampleMs_) return false;
        return (nowMs - lastSampleMs_) >= staleTimeoutMs;
    }

    void noteSensorSample(uint32_t nowMs) {
        lastSampleMs_ = nowMs;
    }

    uint32_t getLastSampleTime() const {
        return lastSampleMs_;
    }

private:
    void recomputeSensorPositionsOnRoads() {
        uint8_t roadCounts[TrafficConfig::VEHICLE_GROUP_COUNT] = { 0, 0, 0 };
        for (uint8_t i = 0; i < TrafficConfig::IR_SENSOR_COUNT; ++i) {
            const uint8_t m = channels_[i].assignedMovement;
            if (m < TrafficConfig::VEHICLE_GROUP_COUNT) {
                channels_[i].sensorPositionOnRoad = roadCounts[m];
                roadCounts[m]++;
            } else {
                channels_[i].sensorPositionOnRoad = 0;
            }
        }
    }

    TrafficConfig::SensorConfig config_;
    IrChannelStatus channels_[TrafficConfig::IR_SENSOR_COUNT];
    RoadPassageStats roadPassages_[TrafficConfig::VEHICLE_GROUP_COUNT];
    uint32_t lastSampleMs_;
};

#endif // VEHICLE_DETECTOR_H
