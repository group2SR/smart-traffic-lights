#ifndef DEMAND_MANAGER_H
#define DEMAND_MANAGER_H

#include <stdint.h>
#include "TrafficConfig.h"
#include "VehicleDetector.h"

// ============================================================================
// DEMAND MANAGER MODULE
// ============================================================================
// Responsibilities:
// - Track vehicle demand (simulated and physical) per vehicle movement:
//   MAIN_ROAD (0), SIDE_ROAD (1), PROTECTED_TURN (2).
// - Track waiting time since demand was first registered while movement is RED.
// - Track number of skipped cycles to prevent starvation.
// - Compute deterministic, testable demand priority scores for ADAPTIVE mode.

struct ApproachDemandState {
    bool     hasDemand;
    bool     simDemandActive;
    bool     simPresenceActive;
    uint8_t  activeSensorCount;
    uint32_t demandStartMs;
    uint32_t waitingTimeMs;
    uint8_t  skippedCycles;
    uint32_t lastScore;
};

class DemandManager {
public:
    DemandManager() {
        config_ = TrafficConfig::DEFAULT_DEMAND_CONFIG;
        reset(0);
    }

    void reset(uint32_t nowMs) {
        for (uint8_t i = 0; i < TrafficConfig::VEHICLE_GROUP_COUNT; ++i) {
            approaches_[i].hasDemand         = false;
            approaches_[i].simDemandActive   = false;
            approaches_[i].simPresenceActive = false;
            approaches_[i].activeSensorCount = 0;
            approaches_[i].demandStartMs     = nowMs;
            approaches_[i].waitingTimeMs     = 0;
            approaches_[i].skippedCycles     = 0;
            approaches_[i].lastScore         = 0;
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

            const bool hwDemand = detector.hasDemandForMovement(m);
            const uint8_t hwCount = detector.countActiveSensorsForMovement(m);

            const bool currentDemand = simulationMode
                                           ? (st.simDemandActive || hwDemand)
                                           : hwDemand;

            st.activeSensorCount = simulationMode
                                       ? (st.simDemandActive ? (hwCount > 0 ? hwCount : 1) : hwCount)
                                       : hwCount;

            const bool isCurrentlyServed = anyVehicleGreenActive && (activeGreenMovement == m);

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

            st.lastScore = calculateScoreForIndex(i);
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
            } else if (approaches_[i].hasDemand) {
                if (approaches_[i].skippedCycles < 255) {
                    approaches_[i].skippedCycles++;
                }
            }
            approaches_[i].lastScore = calculateScoreForIndex(i);
        }
    }

    uint32_t calculateScore(TrafficConfig::MovementId movement) const {
        const uint8_t idx = static_cast<uint8_t>(movement);
        if (idx >= TrafficConfig::VEHICLE_GROUP_COUNT) return 0;
        return calculateScoreForIndex(idx);
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
    uint32_t calculateScoreForIndex(uint8_t idx) const {
        const ApproachDemandState& st = approaches_[idx];
        if (!st.hasDemand && !st.simDemandActive) {
            return 0;
        }
        const uint8_t sensors = (st.activeSensorCount > 0) ? st.activeSensorCount : 1;
        uint32_t score = static_cast<uint32_t>(sensors) * static_cast<uint32_t>(config_.activeSensorWeight);
        if (config_.waitTimeWeightMs > 0) {
            score += (st.waitingTimeMs / config_.waitTimeWeightMs);
        }
        if (st.skippedCycles >= config_.starvationSkipLimit) {
            score += config_.starvationBoostScore;
        }
        return score;
    }

    TrafficConfig::DemandConfig config_;
    ApproachDemandState approaches_[TrafficConfig::VEHICLE_GROUP_COUNT];
};

#endif // DEMAND_MANAGER_H

