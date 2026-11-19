#ifndef DEMAND_MANAGER_H
#define DEMAND_MANAGER_H

#include <stdint.h>
#include "TrafficConfig.h"
#include "VehicleDetector.h"

// ============================================================================
// DEMAND MANAGER MODULE
// ============================================================================
// Responsibilities:
// - Track vehicle demand (simulated and physical across both sensors per road)
//   for each vehicle movement:
//     MAIN_ROAD (0)      : Sensor 1 (GPIO 33) & Sensor 2 (GPIO 34)
//     SIDE_ROAD (1)      : Sensor 1 (GPIO 35) & Sensor 2 (GPIO 36)
//     PROTECTED_TURN (2) : Sensor 1 (GPIO 39) & Sensor 2 (GPIO 15)
// - Track waiting time since demand was first registered while movement is RED.
// - Track number of skipped cycles to prevent starvation.
// - Compute deterministic, testable demand priority scores for ADAPTIVE mode.

struct ApproachDemandState {
    bool     hasDemand;
    bool     simDemandActive;
    bool     simPresenceActive;
    bool     sensor1Active;
    bool     sensor2Active;
    uint8_t  activeSensorCount;
    uint32_t demandStartMs;
    uint32_t waitingTimeMs;
    uint8_t  skippedCycles;
    uint32_t lastScore;

    // Advanced Adaptive Normalized Priority Fields (0..100)
    uint32_t presenceStartMs;
    uint32_t continuousPresenceMs;
    uint32_t totalServices;
    uint8_t  demandScore;
    uint8_t  waitingScore;
    uint8_t  fairnessScore;
    uint8_t  continuousScore;
    uint8_t  priorityScore;
    bool     urgencyActive;
    bool     maxWaitReached;

    // Dual-IR Passage-Counting Telemetry (integrated from VehicleDetector)
    uint32_t confirmedPassages;
    uint16_t rollingPassages;
    uint16_t arrivalRateTenthsPerMin;
    uint16_t incompleteSequences;
    uint32_t lastPassageMs;
    bool     bothOccupied;
    bool     congestionSuspended;
};

class DemandManager {
public:
    DemandManager() : config_(TrafficConfig::DEFAULT_DEMAND_CONFIG) {
        reset(0);
    }

    void reset(uint32_t nowMs) {
        for (uint8_t i = 0; i < TrafficConfig::VEHICLE_GROUP_COUNT; ++i) {
            approaches_[i].hasDemand            = false;
            approaches_[i].simDemandActive      = false;
            approaches_[i].simPresenceActive    = false;
            approaches_[i].sensor1Active        = false;
            approaches_[i].sensor2Active        = false;
            approaches_[i].activeSensorCount    = 0;
            approaches_[i].demandStartMs        = nowMs;
            approaches_[i].waitingTimeMs        = 0;
            approaches_[i].skippedCycles        = 0;
            approaches_[i].lastScore            = 0;
            approaches_[i].presenceStartMs      = nowMs;
            approaches_[i].continuousPresenceMs = 0;
            approaches_[i].totalServices        = 0;
            approaches_[i].demandScore          = 0;
            approaches_[i].waitingScore         = 0;
            approaches_[i].fairnessScore        = 0;
            approaches_[i].continuousScore      = 0;
            approaches_[i].priorityScore           = 0;
            approaches_[i].urgencyActive           = false;
            approaches_[i].maxWaitReached          = false;
            approaches_[i].confirmedPassages       = 0;
            approaches_[i].rollingPassages         = 0;
            approaches_[i].arrivalRateTenthsPerMin = 0;
            approaches_[i].incompleteSequences     = 0;
            approaches_[i].lastPassageMs           = 0;
            approaches_[i].bothOccupied            = false;
            approaches_[i].congestionSuspended     = false;
        }
    }

    bool applyDemandConfig(const TrafficConfig::DemandConfig& cfg) {
        bool valid = true;
        config_ = cfg;
        if (config_.waitTimeWeightMs < 100 || config_.waitTimeWeightMs > 60000) {
            config_.waitTimeWeightMs = TrafficConfig::DEFAULT_WAIT_TIME_WEIGHT_MS;
            valid = false;
        }
        if (config_.activeSensorWeight == 0 || config_.activeSensorWeight > 50) {
            config_.activeSensorWeight = TrafficConfig::DEFAULT_ACTIVE_SENSOR_WEIGHT;
            valid = false;
        }
        if (config_.starvationSkipLimit == 0 || config_.starvationSkipLimit > 20) {
            config_.starvationSkipLimit = TrafficConfig::DEFAULT_STARVATION_SKIP_LIMIT;
            valid = false;
        }
        // Support legacy 5-field aggregate initializers where extended fields are 0
        const bool legacyFiveField =
            (config_.weightDemandPct == 0 && config_.weightWaitingPct == 0 &&
             config_.weightFairnessPct == 0 && config_.weightContinuousPct == 0);
        if (legacyFiveField) {
            config_.weightDemandPct        = TrafficConfig::DEFAULT_WEIGHT_DEMAND_PCT;
            config_.weightWaitingPct       = TrafficConfig::DEFAULT_WEIGHT_WAITING_PCT;
            config_.weightFairnessPct      = TrafficConfig::DEFAULT_WEIGHT_FAIRNESS_PCT;
            config_.weightContinuousPct    = TrafficConfig::DEFAULT_WEIGHT_CONTINUOUS_PCT;
            config_.urgencyWaitThresholdMs = TrafficConfig::DEFAULT_URGENCY_WAIT_MS;
            config_.maxWaitLimitMs         = TrafficConfig::DEFAULT_MAX_WAIT_LIMIT_MS;
            config_.pedUrgencyWaitMs       = TrafficConfig::DEFAULT_PED_URGENCY_WAIT_MS;
            config_.continuousDemandSatMs  = TrafficConfig::DEFAULT_CONTINUOUS_SAT_MS;
        } else {
            const uint16_t sumPct = static_cast<uint16_t>(config_.weightDemandPct) +
                                    static_cast<uint16_t>(config_.weightWaitingPct) +
                                    static_cast<uint16_t>(config_.weightFairnessPct) +
                                    static_cast<uint16_t>(config_.weightContinuousPct);
            if (sumPct != 100U) {
                config_.weightDemandPct     = TrafficConfig::DEFAULT_WEIGHT_DEMAND_PCT;
                config_.weightWaitingPct    = TrafficConfig::DEFAULT_WEIGHT_WAITING_PCT;
                config_.weightFairnessPct   = TrafficConfig::DEFAULT_WEIGHT_FAIRNESS_PCT;
                config_.weightContinuousPct = TrafficConfig::DEFAULT_WEIGHT_CONTINUOUS_PCT;
                valid = false;
            }
            if (config_.urgencyWaitThresholdMs < 5000 || config_.urgencyWaitThresholdMs > 120000) {
                config_.urgencyWaitThresholdMs = TrafficConfig::DEFAULT_URGENCY_WAIT_MS;
                valid = false;
            }
            if (config_.maxWaitLimitMs < config_.urgencyWaitThresholdMs || config_.maxWaitLimitMs > 180000) {
                config_.maxWaitLimitMs = TrafficConfig::DEFAULT_MAX_WAIT_LIMIT_MS;
                valid = false;
            }
            if (config_.pedUrgencyWaitMs < 3000 || config_.pedUrgencyWaitMs > 120000) {
                config_.pedUrgencyWaitMs = TrafficConfig::DEFAULT_PED_URGENCY_WAIT_MS;
                valid = false;
            }
            if (config_.continuousDemandSatMs < 1000 || config_.continuousDemandSatMs > 60000) {
                config_.continuousDemandSatMs = TrafficConfig::DEFAULT_CONTINUOUS_SAT_MS;
                valid = false;
            }
        }
        return valid;
    }

    const TrafficConfig::DemandConfig& getDemandConfig() const { return config_; }

    void setSimulatedDemand(TrafficConfig::MovementId movement, uint32_t nowMs, bool active) {
        const uint8_t idx = static_cast<uint8_t>(movement);
        if (idx >= TrafficConfig::VEHICLE_GROUP_COUNT) return;
        ApproachDemandState& st = approaches_[idx];
        if (active && !st.simDemandActive) {
            st.demandStartMs = nowMs;
        }
        st.simDemandActive   = active;
        st.simPresenceActive = active;
        if (!active) {
            st.waitingTimeMs = 0;
        }
    }

    void clearAllSimulatedDemand() {
        for (uint8_t i = 0; i < TrafficConfig::VEHICLE_GROUP_COUNT; ++i) {
            approaches_[i].simDemandActive   = false;
            approaches_[i].simPresenceActive = false;
        }
    }

    void update(uint32_t nowMs,
                bool simulationMode,
                const VehicleDetector& detector,
                TrafficConfig::MovementId activeGreenMovement,
                bool anyVehicleGreenActive) {
        for (uint8_t i = 0; i < TrafficConfig::VEHICLE_GROUP_COUNT; ++i) {
            const TrafficConfig::MovementId m = static_cast<TrafficConfig::MovementId>(i);
            ApproachDemandState& st = approaches_[i];

            const bool hwDemand   = detector.hasDemandForMovement(m);
            const uint8_t hwCount = detector.countActiveSensorsForMovement(m);
            const RoadPassageStats& ps = detector.getRoadPassageStats(m);
            st.sensor1Active = detector.isRoadSensorDetected(m, 0) || detector.isRoadSensorLatched(m, 0);
            st.sensor2Active = detector.isRoadSensorDetected(m, 1) || detector.isRoadSensorLatched(m, 1);
            st.confirmedPassages       = ps.confirmedPassages;
            st.rollingPassages         = ps.rollingPassages;
            st.arrivalRateTenthsPerMin = ps.arrivalRateTenthsPerMin;
            st.incompleteSequences     = ps.incompleteSequences;
            st.lastPassageMs           = ps.lastPassageMs;
            st.bothOccupied            = ps.bothOccupied;
            st.congestionSuspended     = ps.congestionSuspended;

            const bool currentDemand = simulationMode
                                           ? (st.simDemandActive || hwDemand)
                                           : hwDemand;

            st.activeSensorCount = simulationMode
                                       ? (st.simDemandActive ? (hwCount > 0 ? hwCount : 1) : hwCount)
                                       : hwCount;

            const bool isCurrentlyServed = anyVehicleGreenActive && (activeGreenMovement == m);
            const bool presenceNow = (simulationMode && st.simPresenceActive) ||
                                     detector.hasActivePresenceForMovement(m);

            if (presenceNow) {
                if (st.continuousPresenceMs == 0 && st.presenceStartMs == 0) {
                    st.presenceStartMs = nowMs;
                }
                st.continuousPresenceMs = nowMs - st.presenceStartMs;
            } else {
                st.presenceStartMs      = nowMs;
                st.continuousPresenceMs = 0;
            }

            if (currentDemand && !isCurrentlyServed) {
                if (!st.hasDemand) {
                    st.hasDemand     = true;
                    st.demandStartMs = nowMs;
                    st.waitingTimeMs = 0;
                } else {
                    st.waitingTimeMs = nowMs - st.demandStartMs;
                }
            } else if (isCurrentlyServed) {
                st.hasDemand     = currentDemand;
                st.demandStartMs = nowMs;
                st.waitingTimeMs = 0;
                st.skippedCycles = 0;
            } else {
                st.hasDemand     = false;
                st.waitingTimeMs = 0;
                st.skippedCycles = 0;
            }

            recomputeScoresForIndex(i);
        }
    }

    // Called when a movement completes its green service or another movement is selected
    void notifyPhaseServed(uint32_t nowMs, TrafficConfig::MovementId servedMovement) {
        const uint8_t servedIdx = static_cast<uint8_t>(servedMovement);
        for (uint8_t i = 0; i < TrafficConfig::VEHICLE_GROUP_COUNT; ++i) {
            if (i == servedIdx) {
                approaches_[i].skippedCycles = 0;
                approaches_[i].waitingTimeMs = 0;
                approaches_[i].demandStartMs = nowMs;
                if (approaches_[i].totalServices < 0xFFFFFFFFUL) {
                    approaches_[i].totalServices++;
                }
            } else if (approaches_[i].hasDemand) {
                if (approaches_[i].skippedCycles < 255) {
                    approaches_[i].skippedCycles++;
                }
            }
            recomputeScoresForIndex(i);
        }
    }

    uint32_t calculateScore(TrafficConfig::MovementId movement) const {
        const uint8_t idx = static_cast<uint8_t>(movement);
        if (idx >= TrafficConfig::VEHICLE_GROUP_COUNT) return 0;
        return approaches_[idx].priorityScore;
    }

    uint8_t getNormalizedPriority(TrafficConfig::MovementId movement) const {
        const uint8_t idx = static_cast<uint8_t>(movement);
        if (idx >= TrafficConfig::VEHICLE_GROUP_COUNT) return 0;
        return approaches_[idx].priorityScore;
    }

    bool isApproachAtMaxWait(TrafficConfig::MovementId movement) const {
        const uint8_t idx = static_cast<uint8_t>(movement);
        if (idx >= TrafficConfig::VEHICLE_GROUP_COUNT) return false;
        return approaches_[idx].hasDemand &&
               (approaches_[idx].maxWaitReached ||
                approaches_[idx].skippedCycles >= config_.starvationSkipLimit);
    }

    bool hasDemand(TrafficConfig::MovementId movement) const {
        const uint8_t idx = static_cast<uint8_t>(movement);
        if (idx >= TrafficConfig::VEHICLE_GROUP_COUNT) return false;
        return approaches_[idx].hasDemand || approaches_[idx].simDemandActive;
    }

    bool hasActivePresence(TrafficConfig::MovementId movement, bool simulationMode, const VehicleDetector& detector) const {
        const uint8_t idx = static_cast<uint8_t>(movement);
        if (idx >= TrafficConfig::VEHICLE_GROUP_COUNT) return false;
        if (simulationMode && approaches_[idx].simPresenceActive) {
            return true;
        }
        return detector.hasActivePresenceForMovement(movement);
    }

    const ApproachDemandState& getApproachState(TrafficConfig::MovementId movement) const {
        const uint8_t idx = static_cast<uint8_t>(movement);
        return approaches_[idx < TrafficConfig::VEHICLE_GROUP_COUNT ? idx : 0];
    }

private:
    void recomputeScoresForIndex(uint8_t idx) {
        ApproachDemandState& st = approaches_[idx];
        if (!st.hasDemand && !st.simDemandActive) {
            st.demandScore     = 0;
            st.waitingScore    = 0;
            st.fairnessScore   = 0;
            st.continuousScore = 0;
            st.priorityScore   = 0;
            st.lastScore       = 0;
            st.urgencyActive   = false;
            st.maxWaitReached  = false;
            return;
        }

        // 1. Demand Score (0..100):
        //    - 1 active/latched sensor = 50 (single vehicle always eligible)
        //    - 1 sensor + active rolling passages (>=2) = 75 (sustained flow)
        //    - 2 active/latched sensors or both-occupied congestion = 100
        //    (Passage count is never treated as queue length.)
        const uint8_t sensors = (st.activeSensorCount > 0) ? st.activeSensorCount : 1;
        if (sensors >= 2 || st.bothOccupied) {
            st.demandScore = 100;
        } else if (st.rollingPassages >= 2) {
            st.demandScore = 75;
        } else {
            st.demandScore = 50;
        }

        // 2. Waiting Score (0..100) with 30s urgency increase & 45s max-wait limit
        const uint32_t urgMs = (config_.urgencyWaitThresholdMs > 0) ? config_.urgencyWaitThresholdMs : TrafficConfig::DEFAULT_URGENCY_WAIT_MS;
        const uint32_t maxW  = (config_.maxWaitLimitMs >= urgMs) ? config_.maxWaitLimitMs : TrafficConfig::DEFAULT_MAX_WAIT_LIMIT_MS;
        st.urgencyActive  = (st.waitingTimeMs >= urgMs);
        st.maxWaitReached = (st.waitingTimeMs >= maxW);

        if (st.waitingTimeMs >= maxW) {
            st.waitingScore = 100;
        } else if (st.waitingTimeMs >= urgMs) {
            // Urgency band: 85..100 between 30s and 45s
            const uint32_t span = (maxW > urgMs) ? (maxW - urgMs) : 1;
            const uint32_t over = st.waitingTimeMs - urgMs;
            st.waitingScore = static_cast<uint8_t>(85U + ((over * 15U) / span));
        } else {
            // Normal ramp: 0..70 up to 30s
            st.waitingScore = static_cast<uint8_t>((st.waitingTimeMs * 70U) / urgMs);
        }

        // 3. Fairness Score (0..100) from skipped cycles
        if (st.skippedCycles >= config_.starvationSkipLimit) {
            st.fairnessScore = 100;
        } else if (st.skippedCycles == 1) {
            st.fairnessScore = 60;
        } else {
            st.fairnessScore = 20;
        }

        // 4. Continuous Demand Score (0..100) from sustained presence duration
        const uint32_t satMs = (config_.continuousDemandSatMs > 0) ? config_.continuousDemandSatMs : TrafficConfig::DEFAULT_CONTINUOUS_SAT_MS;
        uint32_t cont = (st.continuousPresenceMs * 100U) / satMs;
        if (cont < 25U) cont = 25U; // Minimum continuous baseline for valid latched demand
        if (cont > 100U) cont = 100U;
        st.continuousScore = static_cast<uint8_t>(cont);

        // Weighted Normalized Priority (0..100)
        uint32_t weighted = (static_cast<uint32_t>(config_.weightDemandPct)     * st.demandScore +
                             static_cast<uint32_t>(config_.weightWaitingPct)    * st.waitingScore +
                             static_cast<uint32_t>(config_.weightFairnessPct)   * st.fairnessScore +
                             static_cast<uint32_t>(config_.weightContinuousPct) * st.continuousScore) / 100U;
        if (weighted > 100U) weighted = 100U;
        st.priorityScore = static_cast<uint8_t>(weighted);
        st.lastScore     = st.priorityScore;
    }

    TrafficConfig::DemandConfig config_;
    ApproachDemandState approaches_[TrafficConfig::VEHICLE_GROUP_COUNT];
};

#endif // DEMAND_MANAGER_H
