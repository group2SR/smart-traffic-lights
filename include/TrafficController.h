#ifndef TRAFFIC_CONTROLLER_H
#define TRAFFIC_CONTROLLER_H

#include <stdint.h>
#include "TrafficConfig.h"
#include "FaultManager.h"
#include "VehicleDetector.h"
#include "DemandManager.h"
#include "PedestrianController.h"
#include "TrafficPhaseManager.h"
#include "SignalOutputDriver.h"
#include "ConfigurationStorage.h"

enum class TrafficState : uint8_t {
    STARTUP_SAFE = 0,
    ALL_RED,
    MAIN_GREEN,
    MAIN_YELLOW,
    SIDE_GREEN,
    SIDE_YELLOW,
    PED_WALK,
    FAULT_SAFE,
    TURN_GREEN,
    TURN_YELLOW
};

enum class OperatingMode : uint8_t {
    HARDWARE_READY = 0,
    SERIAL_SIMULATION
};

enum class NextPhaseTarget : uint8_t {
    MAIN_ROAD = 0,
    SIDE_ROAD,
    PEDESTRIAN,
    PROTECTED_TURN
};

// SignalOutputs is an alias for LogicalSignalOutputs (13 channels; first 8 match v1.1.0)
using SignalOutputs = LogicalSignalOutputs;

// Hardware input adapter structure separating raw GPIO reads from controller logic.
// First 3 fields match v1.1.0 in exact order so legacy 3-element initializers work unchanged.
struct HardwareInputs {
    bool rawPedButtonPinHigh;  // PED 1 pushbutton (GPIO 32)
    bool rawIrSensorPinHigh;   // IR 1 sensor (GPIO 33)
    bool inputsValid;          // True if adapter read is valid

    // Extended v2.0.0 multi-channel inputs (used when hasExtendedChannels == true)
    bool rawPed2ButtonPinHigh; // PED 2 pushbutton (GPIO 4)
    bool hasExtendedChannels;  // Set true when irPinHigh[0..5] & rawPed2ButtonPinHigh are populated
    bool irPinHigh[TrafficConfig::IR_SENSOR_COUNT];
};

// Event callback signatures for decoupled logging / diagnostics
typedef void (*TransitionCallback)(uint32_t nowMs, TrafficState from, TrafficState to, const char* reason);
typedef void (*EventLogCallback)(uint32_t nowMs, const char* eventType, const char* detail);

class TrafficController {
public:
    TrafficController();

    void begin(uint32_t nowMs);
    void update(uint32_t nowMs);

    // ========================================================================
    // TIMING & SUBSYSTEM CONFIGURATION
    // ========================================================================
    bool setTimingConfig(uint32_t nowMs, const TrafficConfig::TimingConfig& cfg);
    const TrafficConfig::TimingConfig& getTimingConfig() const { return timings_; }
    static bool validateAndSanitizeTimings(TrafficConfig::TimingConfig& cfg);

    bool setSensorConfig(uint32_t nowMs, const TrafficConfig::SensorConfig& cfg);
    const TrafficConfig::SensorConfig& getSensorConfig() const { return detector_.getSensorConfig(); }

    bool setDemandConfig(uint32_t nowMs, const TrafficConfig::DemandConfig& cfg);
    const TrafficConfig::DemandConfig& getDemandConfig() const { return demandMgr_.getDemandConfig(); }

    bool setConflictMatrixConfig(uint32_t nowMs, const TrafficConfig::ConflictMatrixConfig& cfg);
    const TrafficConfig::ConflictMatrixConfig& getConflictMatrixConfig() const { return phaseMgr_.getConflictMatrix(); }

    void setProtectedTurnCommissioned(uint32_t nowMs, bool commissioned);
    bool isProtectedTurnCommissioned() const { return phaseMgr_.isProtectedTurnCommissioned(); }

    // ========================================================================
    // PERSISTENT NVS CONFIGURATION (SAVE / LOAD / DEFAULTS)
    // ========================================================================
    PersistentControllerConfig exportPersistentConfig() const;
    bool applyPersistentConfig(uint32_t nowMs, const PersistentControllerConfig& cfg);
    bool saveConfigToNvs(uint32_t nowMs);
    bool loadConfigFromNvs(uint32_t nowMs);
    bool restoreDefaultConfig(uint32_t nowMs);
    ConfigurationStorage& getStorage() { return storage_; }
    const ConfigurationStorage& getStorage() const { return storage_; }

    // ========================================================================
    // OPERATING MODES (AUTO, FIXED, ACTUATED, ADAPTIVE, MANUAL + SIM)
    // ========================================================================
    bool setControlMode(uint32_t nowMs, ControlMode mode);
    ControlMode getControlMode() const { return controlMode_; }

    bool setSimulationMode(uint32_t nowMs, bool enable);
    OperatingMode getMode() const { return mode_; }

    // Manual mode progression (NEXT command) — always enforces YELLOW and ALL_RED clearance
    bool advanceManualStep(uint32_t nowMs);
    bool isManualAdvancePending() const { return manualStepRequested_; }

    // ========================================================================
    // HARDWARE & SIMULATED INPUT PROCESSING
    // ========================================================================
    void processPhysicalInputs(uint32_t nowMs, bool rawPedButtonPinHigh, bool rawIrSensorPinHigh);
    void processAllPhysicalInputs(uint32_t nowMs,
                                  const bool irPinHigh[TrafficConfig::IR_SENSOR_COUNT],
                                  bool rawPed1PinHigh,
                                  bool rawPed2PinHigh);
    void processHardwareAdapter(uint32_t nowMs, const HardwareInputs& inputs);

    // Simulation demand injection (returns false if rejected due to mode or fault)
    bool setSimVehicleDemand(uint32_t nowMs, bool active);
    bool setSimMovementDemand(uint32_t nowMs, TrafficConfig::MovementId movement, bool active);
    bool triggerSimPedestrianRequest(uint32_t nowMs);
    bool triggerSimPedestrianChannel(uint32_t nowMs, uint8_t pedChannelIdx);
    bool clearSimulatedDemand(uint32_t nowMs);

    // Extensible Emergency Priority Interface (never triggered by IR sensors)
    bool requestEmergencyPriority(uint32_t nowMs, TrafficConfig::MovementId movement, const char* detectorType);
    void clearEmergencyPriority(uint32_t nowMs);
    bool isEmergencyPriorityActive() const { return phaseMgr_.isEmergencyPriorityActive(); }

    // ========================================================================
    // FAULT MANAGEMENT & CONTROLLED RECOVERY
    // ========================================================================
    void resetLogical(uint32_t nowMs, const char* reason = "OPERATOR_RESET");
    void triggerFault(uint32_t nowMs, const char* reason, FaultCode code = FaultCode::OPERATOR_INJECTED_FAULT);
    bool clearFaultAndRecover(uint32_t nowMs, const char* reason = "OPERATOR_FAULT_CLEAR");

    // ========================================================================
    // OUTPUT DERIVATION & SAFETY INVARIANTS
    // ========================================================================
    SignalOutputs getOutputs() const;
    static SignalOutputs deriveOutputsForState(TrafficState state);
    static bool verifySafetyInvariants(TrafficState state, const SignalOutputs& out, const char** violationReason = nullptr);

    // ========================================================================
    // STATE, TIMER & SUBSYSTEM INSPECTION
    // ========================================================================
    TrafficState getState() const { return state_; }
    NextPhaseTarget getNextTarget() const { return nextTarget_; }
    uint32_t getStateStartTime() const { return stateStartMs_; }
    uint32_t getStateElapsed(uint32_t nowMs) const { return nowMs - stateStartMs_; }
    uint32_t getSideGapElapsed(uint32_t nowMs) const { return sideGapTimerActive_ ? (nowMs - sideGapStartMs_) : 0; }
    bool isSideGapTimerActive() const { return sideGapTimerActive_; }
    bool isPedestrianClearanceActive() const { return postPedClearanceActive_; }

    bool hasSideVehicleDemand() const;
    bool hasMainVehicleDemand() const;
    bool hasTurnVehicleDemand() const;
    bool hasMovementDemand(TrafficConfig::MovementId movement) const;

    bool hasPedestrianDemand() const { return pedCtrl_.hasAnyDemand(); }
    bool hasPedestrianChannelDemand(uint8_t idx) const { return pedCtrl_.hasDemand(idx); }

    bool isSimVehicleActive() const { return demandMgr_.getApproachState(TrafficConfig::MovementId::SIDE_ROAD).simPresenceActive; }
    bool isHwVehicleDetected() const { return detector_.getChannel(0).stableDetected; }
    bool isHwVehicleLatched() const { return detector_.getChannel(0).latchedDemand; }
    bool isHwIrFaultSuspected() const { return detector_.getChannel(0).stuckFault || detector_.getChannel(0).chatterFault; }
    const char* getFaultReason() const { return faultMgr_.getLastFaultReason(); }

    const FaultManager& getFaultManager() const { return faultMgr_; }
    const VehicleDetector& getVehicleDetector() const { return detector_; }
    const DemandManager& getDemandManager() const { return demandMgr_; }
    const PedestrianController& getPedestrianController() const { return pedCtrl_; }
    const TrafficPhaseManager& getPhaseManager() const { return phaseMgr_; }
    SignalOutputDriver& getOutputDriver() { return outputDriver_; }
    const SignalOutputDriver& getOutputDriver() const { return outputDriver_; }

    // Helpers for string names
    static const char* stateToString(TrafficState s);
    static const char* modeToString(OperatingMode m);
    static const char* controlModeToString(ControlMode m);
    static const char* targetToString(NextPhaseTarget t);

    // Diagnostic callbacks
    void setTransitionCallback(TransitionCallback cb) { transitionCb_ = cb; }
    void setEventLogCallback(EventLogCallback cb) { eventLogCb_ = cb; }

    // Allow configuring whether physical IR sensor(s) are validated
    void setPhysicalIrValidated(bool validated);
    void setPhysicalIrChannelValidated(uint8_t idx, bool validated);
    bool isPhysicalIrValidated() const { return detector_.isChannelValidated(0); }

private:
    void transitionTo(uint32_t nowMs, TrafficState newState, const char* reason);
    void logEvent(uint32_t nowMs, const char* eventType, const char* detail);
    bool isValidTransition(TrafficState from, TrafficState to) const;

    NextPhaseTarget selectNextTargetAfterMain() const;
    NextPhaseTarget selectNextTargetAfterSide() const;
    NextPhaseTarget selectNextTargetAfterTurn() const;
    NextPhaseTarget selectNextTargetAdaptive(TrafficConfig::MovementId currentMovement) const;
    NextPhaseTarget selectNextTargetFixed(TrafficState currentGreen) const;
    NextPhaseTarget selectNextTargetManual(TrafficState currentGreen) const;

    bool hasActivePresenceForCurrentPhase(TrafficConfig::MovementId movement) const;
    void refreshDemandManager(uint32_t nowMs);

    TrafficConfig::TimingConfig timings_;

    TrafficState    state_;
    TrafficState    lastServedNonMain_;
    OperatingMode   mode_;
    ControlMode     controlMode_;
    NextPhaseTarget nextTarget_;

    uint32_t stateStartMs_;
    uint32_t sideGapStartMs_;
    bool     sideGapTimerActive_;
    bool     postPedClearanceActive_;
    bool     manualStepRequested_;

    FaultManager         faultMgr_;
    VehicleDetector      detector_;
    DemandManager        demandMgr_;
    PedestrianController pedCtrl_;
    TrafficPhaseManager  phaseMgr_;
    SignalOutputDriver   outputDriver_;
    ConfigurationStorage storage_;

    TransitionCallback transitionCb_;
    EventLogCallback   eventLogCb_;
};

#endif // TRAFFIC_CONTROLLER_H
