#ifndef TRAFFIC_PHASE_MANAGER_H
#define TRAFFIC_PHASE_MANAGER_H

#include <stdint.h>
#include <string.h>
#include "TrafficConfig.h"

// ============================================================================
// TRAFFIC PHASE & CONFLICT MATRIX MANAGER
// ============================================================================
// Responsibilities:
// - Maintain the explicit, configurable movement-conflict matrix.
// - Enforce that Protected Turn (Vehicle Group 3) remains disabled/uncommissioned
//   until physical junction turning paths are confirmed (protectedTurnCommissioned).
// - Never infer that two movements are compatible simply because they belong to
//   different signal groups.
// - Provide an extensible interface for future emergency-vehicle detection
//   while strictly forbidding activation from ordinary IR presence sensors.

enum class ControlMode : uint8_t {
    AUTO     = 0, // Default automatic mode (vehicle-actuated + fair demand rotation)
    FIXED    = 1, // Fixed-time cyclic scheduling across enabled movements
    ACTUATED = 2, // Vehicle-actuated across 6 IR sensors with min/max green & clear-gap
    ADAPTIVE = 3, // Adaptive demand scoring (sensor count + waiting time + starvation prevention)
    MANUAL   = 4  // Operator step-by-step control (enforces yellow + all-red safety transitions)
};

class TrafficPhaseManager {
public:
    TrafficPhaseManager()
        : conflictMatrix_(TrafficConfig::DEFAULT_CONFLICT_MATRIX),
          emergencyActive_(false),
          emergencyMovement_(TrafficConfig::MovementId::MAIN_ROAD) {}

    bool applyConflictMatrix(const TrafficConfig::ConflictMatrixConfig& cfg) {
        bool valid = true;
        TrafficConfig::ConflictMatrixConfig sanitized = cfg;

        if (!sanitized.rightHandTrafficRule) {
            sanitized.rightHandTrafficRule = true;
            valid = false;
        }

        const size_t n = static_cast<size_t>(TrafficConfig::MovementId::COUNT);
        for (size_t i = 0; i < n; ++i) {
            if (sanitized.conflicts[i][i]) {
                sanitized.conflicts[i][i] = false;
                valid = false;
            }
            for (size_t j = 0; j < n; ++j) {
                if (i != j && sanitized.conflicts[i][j] != sanitized.conflicts[j][i]) {
                    // Symmetry enforcement: if either direction conflicts, both conflict
                    sanitized.conflicts[i][j] = true;
                    sanitized.conflicts[j][i] = true;
                    valid = false;
                }
            }
        }

        // Core physical safety floor: Main Road and Side Road must always conflict;
        // Vehicle movements must always conflict with Pedestrian crossings unless
        // separate verified geometry exists (never allow all-false conflict matrix).
        if (!sanitized.conflicts[0][1] || !sanitized.conflicts[1][0] ||
            !sanitized.conflicts[0][2] || !sanitized.conflicts[1][2] ||
            !sanitized.conflicts[0][3] || !sanitized.conflicts[1][3] ||
            !sanitized.conflicts[2][3] || !sanitized.conflicts[0][4] ||
            !sanitized.conflicts[1][4] || !sanitized.conflicts[2][4]) {
            sanitized = TrafficConfig::DEFAULT_CONFLICT_MATRIX;
            valid = false;
        }

        conflictMatrix_ = sanitized;
        return valid;
    }

    const TrafficConfig::ConflictMatrixConfig& getConflictMatrix() const {
        return conflictMatrix_;
    }

    bool isRightHandTrafficRule() const {
        return conflictMatrix_.rightHandTrafficRule;
    }

    static const TrafficConfig::RhtMovementDescriptor& getMovementDescriptor(TrafficConfig::MovementId m) {
        const size_t idx = static_cast<size_t>(m);
        if (idx < static_cast<size_t>(TrafficConfig::MovementId::COUNT)) {
            return TrafficConfig::RHT_MOVEMENT_DESCRIPTORS[idx];
        }
        return TrafficConfig::RHT_MOVEMENT_DESCRIPTORS[0];
    }

    void setProtectedTurnCommissioned(bool commissioned) {
        conflictMatrix_.protectedTurnCommissioned = commissioned;
    }

    bool isProtectedTurnCommissioned() const {
        return conflictMatrix_.protectedTurnCommissioned;
    }

    bool areMovementsConflicting(TrafficConfig::MovementId a, TrafficConfig::MovementId b) const {
        const size_t ia = static_cast<size_t>(a);
        const size_t ib = static_cast<size_t>(b);
        const size_t n  = static_cast<size_t>(TrafficConfig::MovementId::COUNT);
        if (ia >= n || ib >= n) return true;
        if (ia == ib) return false;
        return conflictMatrix_.conflicts[ia][ib] || conflictMatrix_.conflicts[ib][ia];
    }

    bool isMovementPermitted(TrafficConfig::MovementId candidate,
                             const bool activeMovements[static_cast<size_t>(TrafficConfig::MovementId::COUNT)]) const {
        if (!conflictMatrix_.rightHandTrafficRule) {
            return false;
        }
        if (candidate == TrafficConfig::MovementId::PROTECTED_TURN && !conflictMatrix_.protectedTurnCommissioned) {
            return false;
        }
        const size_t n = static_cast<size_t>(TrafficConfig::MovementId::COUNT);
        for (size_t i = 0; i < n; ++i) {
            if (activeMovements[i] && i != static_cast<size_t>(candidate)) {
                if (areMovementsConflicting(candidate, static_cast<TrafficConfig::MovementId>(i))) {
                    return false;
                }
            }
        }
        return true;
    }

    bool verifyActiveMovementsConflictFree(
        const bool activeMovements[static_cast<size_t>(TrafficConfig::MovementId::COUNT)]) const {
        if (!conflictMatrix_.rightHandTrafficRule) {
            return false;
        }
        const size_t n = static_cast<size_t>(TrafficConfig::MovementId::COUNT);
        for (size_t i = 0; i < n; ++i) {
            if (activeMovements[i]) {
                if (!isMovementPermitted(static_cast<TrafficConfig::MovementId>(i), activeMovements)) {
                    return false;
                }
            }
        }
        return true;
    }

    // Extensible Emergency Priority Interface:
    // Explicitly rejects activation from ordinary IR sensors ("IR_SENSOR"), because
    // IR obstacle sensors are ordinary presence detectors, not emergency detectors.
    bool requestEmergencyPriority(TrafficConfig::MovementId movement, const char* detectorType) {
        if (!detectorType || strcmp(detectorType, "IR_SENSOR") == 0) {
            return false;
        }
        if (static_cast<uint8_t>(movement) >= TrafficConfig::VEHICLE_GROUP_COUNT) {
            return false;
        }
        if (movement == TrafficConfig::MovementId::PROTECTED_TURN && !conflictMatrix_.protectedTurnCommissioned) {
            return false;
        }
        emergencyActive_   = true;
        emergencyMovement_ = movement;
        return true;
    }

    void clearEmergencyPriority() {
        emergencyActive_ = false;
    }

    bool isEmergencyPriorityActive() const { return emergencyActive_; }
    TrafficConfig::MovementId getEmergencyMovement() const { return emergencyMovement_; }

    static const char* controlModeToString(ControlMode m) {
        switch (m) {
            case ControlMode::AUTO:     return "AUTO";
            case ControlMode::FIXED:    return "FIXED";
            case ControlMode::ACTUATED: return "ACTUATED";
            case ControlMode::ADAPTIVE: return "ADAPTIVE";
            case ControlMode::MANUAL:   return "MANUAL";
            default:                    return "UNKNOWN";
        }
    }

private:
    TrafficConfig::ConflictMatrixConfig conflictMatrix_;
    bool                                emergencyActive_;
    TrafficConfig::MovementId           emergencyMovement_;
};

#endif // TRAFFIC_PHASE_MANAGER_H
