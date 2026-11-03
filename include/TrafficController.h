#ifndef TRAFFIC_CONTROLLER_H
#define TRAFFIC_CONTROLLER_H

#include <stdint.h>
#include "TrafficConfig.h"

enum class TrafficState : uint8_t {
    STARTUP_SAFE = 0,
    ALL_RED,
    MAIN_GREEN,
    MAIN_YELLOW,
    SIDE_GREEN,
    SIDE_YELLOW,
    PED_WALK,
    FAULT_SAFE
};

enum class OperatingMode : uint8_t {
    HARDWARE_READY = 0,
    SERIAL_SIMULATION
};

enum class NextPhaseTarget : uint8_t {
    MAIN_ROAD = 0,
    SIDE_ROAD,
    PEDESTRIAN
};

struct SignalOutputs {
    bool mainGreen;
    bool mainYellow;
    bool mainRed;
    bool sideGreen;
    bool sideYellow;
    bool sideRed;
    bool pedWalk;
    bool pedDontWalk;
};

// Hardware input adapter structure separating raw GPIO reads from controller logic
struct HardwareInputs {
    bool rawPedButtonPinHigh;
    bool rawIrSensorPinHigh;
    bool inputsValid;
};

// Event callback signatures for decoupled logging / diagnostics
typedef void (*TransitionCallback)(uint32_t nowMs, TrafficState from, TrafficState to, const char* reason);
typedef void (*EventLogCallback)(uint32_t nowMs, const char* eventType, const char* detail);

class TrafficController {
public:
    TrafficController();

    void begin(uint32_t nowMs);
    void update(uint32_t nowMs);

    // Timing configuration & validation (falls back to safe defaults if invalid)
    bool setTimingConfig(uint32_t nowMs, const TrafficConfig::TimingConfig& cfg);
    const TrafficConfig::TimingConfig& getTimingConfig() const { return timings_; }
    static bool validateAndSanitizeTimings(TrafficConfig::TimingConfig& cfg);

    // Hardware input processing (non-blocking, called during HARDWARE_READY)
    void processPhysicalInputs(uint32_t nowMs, bool rawPedButtonPinHigh, bool rawIrSensorPinHigh);
    void processHardwareAdapter(uint32_t nowMs, const HardwareInputs& inputs);

    // Mode management
    bool setSimulationMode(uint32_t nowMs, bool enable);
    OperatingMode getMode() const { return mode_; }

    // Simulation demand injection (returns false if rejected due to mode or fault)
    bool setSimVehicleDemand(uint32_t nowMs, bool active);
    bool triggerSimPedestrianRequest(uint32_t nowMs);
    bool clearSimulatedDemand(uint32_t nowMs);

    // Controlled logical reset and fault injection/handling
    void resetLogical(uint32_t nowMs, const char* reason = "OPERATOR_RESET");
    void triggerFault(uint32_t nowMs, const char* reason);

    // Output derivation & safety verification
    SignalOutputs getOutputs() const;
    static SignalOutputs deriveOutputsForState(TrafficState state);
    static bool verifySafetyInvariants(TrafficState state, const SignalOutputs& out, const char** violationReason = nullptr);

    // State and timer inspection
    TrafficState getState() const { return state_; }
    NextPhaseTarget getNextTarget() const { return nextTarget_; }
    uint32_t getStateStartTime() const { return stateStartMs_; }
    uint32_t getStateElapsed(uint32_t nowMs) const { return nowMs - stateStartMs_; }
    uint32_t getSideGapElapsed(uint32_t nowMs) const { return sideGapTimerActive_ ? (nowMs - sideGapStartMs_) : 0; }
    bool isSideGapTimerActive() const { return sideGapTimerActive_; }

    bool hasSideVehicleDemand() const;
    bool hasPedestrianDemand() const { return pedRequestLatched_; }
    bool isSimVehicleActive() const { return simVehiclePresent_; }
    bool isHwVehicleDetected() const { return hwIrStableDetected_; }
    bool isHwVehicleLatched() const { return hwVehicleLatched_; }
    bool isHwIrFaultSuspected() const { return hwIrStuckFault_; }
    const char* getFaultReason() const { return faultReason_; }

    // Helpers for string names
    static const char* stateToString(TrafficState s);
    static const char* modeToString(OperatingMode m);
    static const char* targetToString(NextPhaseTarget t);

    // Diagnostic callbacks
    void setTransitionCallback(TransitionCallback cb) { transitionCb_ = cb; }
    void setEventLogCallback(EventLogCallback cb) { eventLogCb_ = cb; }

    // Allow configuring whether physical IR sensor is validated (for hardware bringup / unit tests)
    void setPhysicalIrValidated(bool validated) { physicalIrValidated_ = validated; }
    bool isPhysicalIrValidated() const { return physicalIrValidated_; }

private:
    void transitionTo(uint32_t nowMs, TrafficState newState, const char* reason);
    void logEvent(uint32_t nowMs, const char* eventType, const char* detail);
    bool isValidTransition(TrafficState from, TrafficState to) const;
    NextPhaseTarget selectNextTargetAfterMain() const;

    TrafficConfig::TimingConfig timings_;

    TrafficState state_;
    TrafficState lastServedNonMain_;
    OperatingMode mode_;
    NextPhaseTarget nextTarget_;

    uint32_t stateStartMs_;
    uint32_t sideGapStartMs_;
    bool sideGapTimerActive_;

    // Latched / active demands
    bool pedRequestLatched_;
    bool pedRequestFromSim_;

    bool simVehiclePresent_;
    bool hwVehicleLatched_;

    // Physical button debounce state
    bool hwBtnLastRawPressed_;
    bool hwBtnStablePressed_;
    bool hwBtnAwaitRelease_;
    uint32_t hwBtnLastChangeMs_;

    // Physical IR sensor stability & stuck-sensor supervision state
    bool hwIrLastRawDetected_;
    bool hwIrStableDetected_;
    bool hwIrStuckFault_;
    uint32_t hwIrLastChangeMs_;
    uint32_t hwIrActiveStartMs_;
    bool physicalIrValidated_;

    const char* faultReason_;

    TransitionCallback transitionCb_;
    EventLogCallback eventLogCb_;
};

#endif // TRAFFIC_CONTROLLER_H
