#include "TrafficController.h"

TrafficController::TrafficController()
    : timings_(TrafficConfig::DEFAULT_TIMING_CONFIG),
      state_(TrafficState::STARTUP_SAFE),
      lastServedNonMain_(TrafficState::STARTUP_SAFE),
      mode_(OperatingMode::HARDWARE_READY),
      controlMode_(ControlMode::AUTO),
      nextTarget_(NextPhaseTarget::MAIN_ROAD),
      stateStartMs_(0),
      sideGapStartMs_(0),
      sideGapTimerActive_(false),
      postPedClearanceActive_(false),
      manualStepRequested_(false),
      transitionCb_(nullptr),
      eventLogCb_(nullptr) {}

void TrafficController::begin(uint32_t nowMs) {
    validateAndSanitizeTimings(timings_);

    state_                  = TrafficState::STARTUP_SAFE;
    lastServedNonMain_      = TrafficState::STARTUP_SAFE;
    mode_                   = OperatingMode::HARDWARE_READY;
    controlMode_            = ControlMode::AUTO;
    nextTarget_             = NextPhaseTarget::MAIN_ROAD;
    stateStartMs_           = nowMs;
    sideGapStartMs_         = nowMs;
    sideGapTimerActive_     = false;
    postPedClearanceActive_ = false;
    manualStepRequested_    = false;

    faultMgr_.reset(nowMs);
    detector_.reset(nowMs);
    demandMgr_.reset(nowMs);
    pedCtrl_.reset(nowMs);
    phaseMgr_.clearEmergencyPriority();
}

bool TrafficController::validateAndSanitizeTimings(TrafficConfig::TimingConfig& cfg) {
    bool allValid = true;
    const auto checkRange = [&](uint32_t& val, uint32_t fallback) {
        if (val < TrafficConfig::MIN_ALLOWED_INTERVAL_MS || val > TrafficConfig::MAX_ALLOWED_INTERVAL_MS) {
            val = fallback;
            allValid = false;
        }
    };

    // Detect whether the caller used a legacy 9-field aggregate initializer
    // (where fields 10..16 were zero-initialized by the compiler).
    const bool legacyNineFieldInit =
        (cfg.mainMaxGreenMs == 0 && cfg.mainClearGapMs == 0 &&
         cfg.turnMinGreenMs == 0 && cfg.turnMaxGreenMs == 0 &&
         cfg.turnClearGapMs == 0 && cfg.turnYellowMs == 0 &&
         cfg.pedClearanceMs == 0);

    // Validate core 9 fields
    checkRange(cfg.startupSafeMs,  TrafficConfig::STARTUP_SAFE_MS);
    checkRange(cfg.allRedMs,       TrafficConfig::ALL_RED_MS);
    checkRange(cfg.mainMinGreenMs, TrafficConfig::MAIN_MIN_GREEN_MS);
    checkRange(cfg.mainYellowMs,   TrafficConfig::MAIN_YELLOW_MS);
    checkRange(cfg.sideMinGreenMs, TrafficConfig::SIDE_MIN_GREEN_MS);
    checkRange(cfg.sideMaxGreenMs, TrafficConfig::SIDE_MAX_GREEN_MS);
    checkRange(cfg.sideClearGapMs, TrafficConfig::SIDE_CLEAR_GAP_MS);
    checkRange(cfg.sideYellowMs,   TrafficConfig::SIDE_YELLOW_MS);
    checkRange(cfg.pedWalkMs,      TrafficConfig::PED_WALK_MS);

    if (cfg.sideMaxGreenMs < cfg.sideMinGreenMs) {
        cfg.sideMinGreenMs = TrafficConfig::SIDE_MIN_GREEN_MS;
        cfg.sideMaxGreenMs = TrafficConfig::SIDE_MAX_GREEN_MS;
        allValid = false;
    }

    if (legacyNineFieldInit) {
        cfg.mainMaxGreenMs = (cfg.mainMinGreenMs > TrafficConfig::MAIN_MAX_GREEN_MS)
                                 ? (cfg.mainMinGreenMs * 2)
                                 : TrafficConfig::MAIN_MAX_GREEN_MS;
        cfg.mainClearGapMs = TrafficConfig::MAIN_CLEAR_GAP_MS;
        cfg.turnMinGreenMs = TrafficConfig::TURN_MIN_GREEN_MS;
        cfg.turnMaxGreenMs = TrafficConfig::TURN_MAX_GREEN_MS;
        cfg.turnClearGapMs = TrafficConfig::TURN_CLEAR_GAP_MS;
        cfg.turnYellowMs   = TrafficConfig::TURN_YELLOW_MS;
        cfg.pedClearanceMs = cfg.allRedMs;
    } else {
        checkRange(cfg.mainMaxGreenMs, TrafficConfig::MAIN_MAX_GREEN_MS);
        checkRange(cfg.mainClearGapMs, TrafficConfig::MAIN_CLEAR_GAP_MS);
        checkRange(cfg.turnMinGreenMs, TrafficConfig::TURN_MIN_GREEN_MS);
        checkRange(cfg.turnMaxGreenMs, TrafficConfig::TURN_MAX_GREEN_MS);
        checkRange(cfg.turnClearGapMs, TrafficConfig::TURN_CLEAR_GAP_MS);
        checkRange(cfg.turnYellowMs,   TrafficConfig::TURN_YELLOW_MS);
        checkRange(cfg.pedClearanceMs, TrafficConfig::PED_CLEARANCE_MS);

        if (cfg.mainMaxGreenMs < cfg.mainMinGreenMs) {
            cfg.mainMinGreenMs = TrafficConfig::MAIN_MIN_GREEN_MS;
            cfg.mainMaxGreenMs = TrafficConfig::MAIN_MAX_GREEN_MS;
            allValid = false;
        }
        if (cfg.turnMaxGreenMs < cfg.turnMinGreenMs) {
            cfg.turnMinGreenMs = TrafficConfig::TURN_MIN_GREEN_MS;
            cfg.turnMaxGreenMs = TrafficConfig::TURN_MAX_GREEN_MS;
            allValid = false;
        }
    }

    return allValid;
}

bool TrafficController::setTimingConfig(uint32_t nowMs, const TrafficConfig::TimingConfig& cfg) {
    TrafficConfig::TimingConfig sanitized = cfg;
    const bool valid = validateAndSanitizeTimings(sanitized);
    timings_ = sanitized;
    if (!valid) {
        faultMgr_.recordNonFatalFault(nowMs, FaultCode::INVALID_CONFIGURATION,
                                      "Invalid timing configuration sanitized to safe defaults");
        logEvent(nowMs, "TIMING_FALLBACK",
                 "Invalid timing configuration detected; fell back to documented safe defaults");
    } else {
        logEvent(nowMs, "TIMING_UPDATED", "Validated timing configuration applied");
    }
    return valid;
}

bool TrafficController::setSensorConfig(uint32_t nowMs, const TrafficConfig::SensorConfig& cfg) {
    const bool valid = detector_.applySensorConfig(cfg);
    if (!valid) {
        faultMgr_.recordNonFatalFault(nowMs, FaultCode::INVALID_CONFIGURATION,
                                      "Invalid sensor configuration sanitized to safe defaults");
        logEvent(nowMs, "SENSOR_CONFIG_FALLBACK", "Invalid sensor configuration sanitized");
    } else {
        logEvent(nowMs, "SENSOR_CONFIG_UPDATED", "Validated sensor configuration applied");
    }
    return valid;
}

bool TrafficController::setDemandConfig(uint32_t nowMs, const TrafficConfig::DemandConfig& cfg) {
    const bool valid = demandMgr_.applyDemandConfig(cfg);
    if (!valid) {
        faultMgr_.recordNonFatalFault(nowMs, FaultCode::INVALID_CONFIGURATION,
                                      "Invalid demand configuration sanitized to safe defaults");
        logEvent(nowMs, "DEMAND_CONFIG_FALLBACK", "Invalid demand configuration sanitized");
    } else {
        logEvent(nowMs, "DEMAND_CONFIG_UPDATED", "Validated demand configuration applied");
    }
    return valid;
}

bool TrafficController::setConflictMatrixConfig(uint32_t nowMs, const TrafficConfig::ConflictMatrixConfig& cfg) {
    const bool valid = phaseMgr_.applyConflictMatrix(cfg);
    if (!valid) {
        faultMgr_.recordNonFatalFault(nowMs, FaultCode::INVALID_CONFIGURATION,
                                      "Unsafe conflict matrix rejected; restored default Keep-Right conflict matrix");
        logEvent(nowMs, "CONFLICT_MATRIX_FALLBACK",
                 "Unsafe conflict matrix rejected; restored default Keep-Right conflict matrix");
    } else {
        logEvent(nowMs, "CONFLICT_MATRIX_UPDATED", "Validated Keep-Right conflict matrix applied");
    }
    return valid;
}

void TrafficController::setProtectedTurnCommissioned(uint32_t nowMs, bool commissioned) {
    phaseMgr_.setProtectedTurnCommissioned(commissioned);
    if (!commissioned && (state_ == TrafficState::TURN_GREEN || state_ == TrafficState::TURN_YELLOW)) {
        nextTarget_ = NextPhaseTarget::MAIN_ROAD;
        if (state_ == TrafficState::TURN_GREEN) {
            transitionTo(nowMs, TrafficState::TURN_YELLOW, "TURN_DECOMMISSIONED_SAFE_EXIT");
        }
    }
    logEvent(nowMs, "PROTECTED_TURN_COMMISSION",
             commissioned ? "Protected turn commissioned" : "Protected turn disabled/uncommissioned");
}

PersistentControllerConfig TrafficController::exportPersistentConfig() const {
    PersistentControllerConfig cfg = ConfigurationStorage::makeDefaultConfig();
    cfg.controlMode           = static_cast<uint8_t>(controlMode_);
    cfg.vehicleDriverType     = static_cast<uint8_t>(outputDriver_.getVehicleDriverType());
    cfg.pedDriverType         = static_cast<uint8_t>(outputDriver_.getPedestrianDriverType());
    cfg.vehicleDriverVerified = outputDriver_.isVehicleDriverVerified();
    cfg.pedDriverVerified     = outputDriver_.isPedestrianDriverVerified();
    cfg.timings               = timings_;
    cfg.sensors               = detector_.getSensorConfig();
    cfg.demand                = demandMgr_.getDemandConfig();
    cfg.conflictMatrix        = phaseMgr_.getConflictMatrix();
    cfg.checksum              = ConfigurationStorage::computeChecksum(cfg);
    return cfg;
}

bool TrafficController::applyPersistentConfig(uint32_t nowMs, const PersistentControllerConfig& cfg) {
    if (!ConfigurationStorage::validateConfig(cfg)) {
        const PersistentControllerConfig def = ConfigurationStorage::makeDefaultConfig();
        setTimingConfig(nowMs, def.timings);
        setSensorConfig(nowMs, def.sensors);
        setDemandConfig(nowMs, def.demand);
        setConflictMatrixConfig(nowMs, def.conflictMatrix);
        setControlMode(nowMs, static_cast<ControlMode>(def.controlMode));
        outputDriver_.setVehicleDriverConfig(
            static_cast<TrafficConfig::OutputDriverType>(def.vehicleDriverType),
            def.vehicleDriverVerified);
        outputDriver_.setPedestrianDriverConfig(
            static_cast<TrafficConfig::OutputDriverType>(def.pedDriverType),
            def.pedDriverVerified);
        faultMgr_.recordNonFatalFault(nowMs, FaultCode::INVALID_CONFIGURATION,
                                      "Corrupted/invalid persistent config rejected; defaults applied");
        logEvent(nowMs, "CONFIG_FALLBACK", "Invalid persistent configuration; fell back to safe defaults");
        return false;
    }

    setTimingConfig(nowMs, cfg.timings);
    setSensorConfig(nowMs, cfg.sensors);
    setDemandConfig(nowMs, cfg.demand);
    setConflictMatrixConfig(nowMs, cfg.conflictMatrix);
    setControlMode(nowMs, static_cast<ControlMode>(cfg.controlMode));
    outputDriver_.setVehicleDriverConfig(
        static_cast<TrafficConfig::OutputDriverType>(cfg.vehicleDriverType),
        cfg.vehicleDriverVerified);
    outputDriver_.setPedestrianDriverConfig(
        static_cast<TrafficConfig::OutputDriverType>(cfg.pedDriverType),
        cfg.pedDriverVerified);
    logEvent(nowMs, "CONFIG_APPLIED", "Persistent configuration validated and applied");
    return true;
}

bool TrafficController::saveConfigToNvs(uint32_t nowMs) {
    const PersistentControllerConfig cfg = exportPersistentConfig();
    const bool ok = storage_.save(cfg);
    logEvent(nowMs, ok ? "NVS_SAVED" : "NVS_SAVE_FAILED",
             ok ? "Configuration saved to NVS" : "Failed to save configuration to NVS");
    return ok;
}

bool TrafficController::loadConfigFromNvs(uint32_t nowMs) {
    PersistentControllerConfig loaded;
    const bool ok = storage_.load(loaded);
    applyPersistentConfig(nowMs, loaded);
    if (!ok) {
        faultMgr_.recordNonFatalFault(nowMs, FaultCode::INVALID_CONFIGURATION,
                                      "NVS load failed or corrupted; restored safe defaults");
    }
    return ok;
}

bool TrafficController::restoreDefaultConfig(uint32_t nowMs) {
    PersistentControllerConfig def;
    const bool ok = storage_.resetToDefaults(def);
    applyPersistentConfig(nowMs, def);
    logEvent(nowMs, "DEFAULTS_RESTORED", "Controller configuration restored to safe defaults");
    return ok;
}

bool TrafficController::setControlMode(uint32_t nowMs, ControlMode mode) {
    if (state_ == TrafficState::FAULT_SAFE) {
        logEvent(nowMs, "COMMAND_REJECTED", "Cannot change control mode while in FAULT_SAFE");
        return false;
    }
    if (static_cast<uint8_t>(mode) > static_cast<uint8_t>(ControlMode::MANUAL)) {
        return false;
    }
    controlMode_         = mode;
    manualStepRequested_ = false;
    logEvent(nowMs, "CONTROL_MODE_CHANGE", controlModeToString(mode));
    return true;
}

bool TrafficController::advanceManualStep(uint32_t nowMs) {
    if (state_ == TrafficState::FAULT_SAFE) {
        logEvent(nowMs, "COMMAND_REJECTED", "Cannot advance manual step in FAULT_SAFE");
        return false;
    }
    if (controlMode_ != ControlMode::MANUAL) {
        logEvent(nowMs, "COMMAND_REJECTED", "NEXT command requires MODE MANUAL");
        return false;
    }
    manualStepRequested_ = true;
    logEvent(nowMs, "MANUAL_STEP_REQUESTED", "Operator requested phase advance via safe clearance");
    return true;
}

void TrafficController::setPhysicalIrValidated(bool validated) {
    detector_.setAllValidated(validated);
}

void TrafficController::setPhysicalIrChannelValidated(uint8_t idx, bool validated) {
    detector_.setChannelValidated(idx, validated);
}

bool TrafficController::hasMovementDemand(TrafficConfig::MovementId movement) const {
    if (movement == TrafficConfig::MovementId::PROTECTED_TURN && !phaseMgr_.isProtectedTurnCommissioned()) {
        return false;
    }
    const bool simActive = demandMgr_.getApproachState(movement).simDemandActive;
    if (mode_ == OperatingMode::SERIAL_SIMULATION) {
        return simActive || detector_.hasDemandForMovement(movement);
    }
    return detector_.hasDemandForMovement(movement);
}

bool TrafficController::hasSideVehicleDemand() const {
    return hasMovementDemand(TrafficConfig::MovementId::SIDE_ROAD);
}

bool TrafficController::hasMainVehicleDemand() const {
    return hasMovementDemand(TrafficConfig::MovementId::MAIN_ROAD);
}

bool TrafficController::hasTurnVehicleDemand() const {
    return hasMovementDemand(TrafficConfig::MovementId::PROTECTED_TURN);
}

bool TrafficController::hasActivePresenceForCurrentPhase(TrafficConfig::MovementId movement) const {
    return demandMgr_.hasActivePresence(movement, mode_ == OperatingMode::SERIAL_SIMULATION, detector_);
}

void TrafficController::refreshDemandManager(uint32_t nowMs) {
    TrafficConfig::MovementId activeMove = TrafficConfig::MovementId::MAIN_ROAD;
    bool anyGreen = false;
    if (state_ == TrafficState::MAIN_GREEN) {
        activeMove = TrafficConfig::MovementId::MAIN_ROAD;
        anyGreen   = true;
    } else if (state_ == TrafficState::SIDE_GREEN) {
        activeMove = TrafficConfig::MovementId::SIDE_ROAD;
        anyGreen   = true;
    } else if (state_ == TrafficState::TURN_GREEN) {
        activeMove = TrafficConfig::MovementId::PROTECTED_TURN;
        anyGreen   = true;
    }
    demandMgr_.update(nowMs, mode_ == OperatingMode::SERIAL_SIMULATION, detector_, activeMove, anyGreen);
}

void TrafficController::processHardwareAdapter(uint32_t nowMs, const HardwareInputs& inputs) {
    if (!inputs.inputsValid) {
        detector_.clearInvalidAdapterState(nowMs);
        pedCtrl_.clearInvalidAdapterState(nowMs);
        faultMgr_.recordNonFatalFault(nowMs, FaultCode::INVALID_SENSOR_STATE,
                                      "Invalid hardware input adapter frame ignored");
        return;
    }
    if (inputs.hasExtendedChannels) {
        processAllPhysicalInputs(nowMs, inputs.irPinHigh, inputs.rawPedButtonPinHigh, inputs.rawPed2ButtonPinHigh);
    } else {
        processPhysicalInputs(nowMs, inputs.rawPedButtonPinHigh, inputs.rawIrSensorPinHigh);
    }
}

void TrafficController::processPhysicalInputs(uint32_t nowMs, bool rawPedButtonPinHigh, bool rawIrSensorPinHigh) {
    const bool irPins[TrafficConfig::IR_SENSOR_COUNT] = {
        rawIrSensorPinHigh, true, true, true, true, true
    };
    processAllPhysicalInputs(nowMs, irPins, rawPedButtonPinHigh, true);
}

void TrafficController::processAllPhysicalInputs(uint32_t nowMs,
                                                 const bool irPinHigh[TrafficConfig::IR_SENSOR_COUNT],
                                                 bool rawPed1PinHigh,
                                                 bool rawPed2PinHigh) {
    if (mode_ != OperatingMode::HARDWARE_READY || state_ == TrafficState::FAULT_SAFE) {
        return;
    }

    const TrafficConfig::SensorConfig& scfg = detector_.getSensorConfig();

    // 1. Update both pedestrian pushbuttons independently
    const bool rawPedPins[TrafficConfig::PED_BUTTON_COUNT] = { rawPed1PinHigh, rawPed2PinHigh };
    for (uint8_t b = 0; b < TrafficConfig::PED_BUTTON_COUNT; ++b) {
        bool alreadyPending = false;
        const bool newlyLatched = pedCtrl_.updateButtonChannel(
            b, nowMs, rawPedPins[b], scfg.pedDebounceMs, scfg.maxInputStuckMs, faultMgr_, &alreadyPending);
        if (newlyLatched) {
            logEvent(nowMs, "PED_REQUEST_LATCHED",
                     (b == 0) ? "Physical pedestrian button 1 press (debounced)"
                              : "Physical pedestrian button 2 press (debounced)");
        } else if (alreadyPending) {
            logEvent(nowMs, "PED_REQUEST_PENDING", "Physical press ignored; request already latched");
        }
    }

    // 2. Update all 6 IR vehicle sensors independently
    for (uint8_t i = 0; i < TrafficConfig::IR_SENSOR_COUNT; ++i) {
        const bool wasStuck = detector_.getChannel(i).stuckFault;
        const bool newArrival = detector_.updateChannel(i, nowMs, irPinHigh[i], faultMgr_);
        const bool nowStuck = detector_.getChannel(i).stuckFault;

        if (newArrival) {
            logEvent(nowMs, "DEMAND_REGISTERED", "Physical IR vehicle detected");
        } else if (!wasStuck && nowStuck) {
            logEvent(nowMs, "SENSOR_FAULT_SUSPECTED",
                     "IR sensor active continuously > maxInputStuckMs; suppressing stuck input");
        } else if (wasStuck && !nowStuck) {
            logEvent(nowMs, "SENSOR_RECOVERED", "Physical IR sensor cleared after stuck state");
        }
    }

    refreshDemandManager(nowMs);
}

bool TrafficController::setSimulationMode(uint32_t nowMs, bool enable) {
    if (state_ == TrafficState::FAULT_SAFE) {
        logEvent(nowMs, "COMMAND_REJECTED", "Cannot switch mode while in FAULT_SAFE; use RESET first");
        return false;
    }

    if (enable) {
        if (mode_ == OperatingMode::SERIAL_SIMULATION) {
            logEvent(nowMs, "MODE_CHANGE", "Already in SERIAL_SIMULATION mode");
            return true;
        }
        mode_ = OperatingMode::SERIAL_SIMULATION;
        logEvent(nowMs, "MODE_CHANGE", "Switched to SERIAL_SIMULATION mode");
        return true;
    } else {
        if (mode_ == OperatingMode::HARDWARE_READY) {
            logEvent(nowMs, "MODE_CHANGE", "Already in HARDWARE_READY mode");
            return true;
        }

        const bool hadSimVehicle =
            demandMgr_.getApproachState(TrafficConfig::MovementId::SIDE_ROAD).simDemandActive ||
            demandMgr_.getApproachState(TrafficConfig::MovementId::MAIN_ROAD).simDemandActive ||
            demandMgr_.getApproachState(TrafficConfig::MovementId::PROTECTED_TURN).simDemandActive;
        const bool hadSimPed = pedCtrl_.clearSimulatedRequests();

        demandMgr_.clearAllSimulatedDemand();
        detector_.reset(nowMs);
        mode_ = OperatingMode::HARDWARE_READY;

        if (hadSimVehicle && hadSimPed) {
            logEvent(nowMs, "MODE_CHANGE",
                     "Switched to HARDWARE_READY; discarded simulated VEHICLE and PEDESTRIAN demands (NOTE: physical sensors unverified)");
        } else if (hadSimVehicle) {
            logEvent(nowMs, "MODE_CHANGE",
                     "Switched to HARDWARE_READY; discarded simulated VEHICLE demand (NOTE: physical sensors unverified)");
        } else if (hadSimPed) {
            logEvent(nowMs, "MODE_CHANGE",
                     "Switched to HARDWARE_READY; discarded simulated PEDESTRIAN demand (NOTE: physical sensors unverified)");
        } else {
            logEvent(nowMs, "MODE_CHANGE",
                     "Switched to HARDWARE_READY; no simulated demands were active (NOTE: physical sensors unverified)");
        }

        // Safeguard: If an active non-main phase was driven by simulation, exit safely via clearance
        if (state_ == TrafficState::SIDE_GREEN) {
            nextTarget_ = NextPhaseTarget::MAIN_ROAD;
            transitionTo(nowMs, TrafficState::SIDE_YELLOW, "SIM_OFF_SAFE_RECOVERY");
        } else if (state_ == TrafficState::TURN_GREEN) {
            nextTarget_ = NextPhaseTarget::MAIN_ROAD;
            transitionTo(nowMs, TrafficState::TURN_YELLOW, "SIM_OFF_SAFE_RECOVERY");
        } else if (state_ == TrafficState::PED_WALK) {
            nextTarget_ = NextPhaseTarget::MAIN_ROAD;
            transitionTo(nowMs, TrafficState::ALL_RED, "SIM_OFF_SAFE_RECOVERY");
        } else if (state_ == TrafficState::MAIN_YELLOW || state_ == TrafficState::ALL_RED) {
            nextTarget_ = NextPhaseTarget::MAIN_ROAD;
        }

        refreshDemandManager(nowMs);
        return true;
    }
}

bool TrafficController::setSimMovementDemand(uint32_t nowMs, TrafficConfig::MovementId movement, bool active) {
    if (mode_ != OperatingMode::SERIAL_SIMULATION) {
        logEvent(nowMs, "COMMAND_REJECTED", "Simulation command rejected in HARDWARE_READY mode (use SIM ON)");
        return false;
    }
    if (state_ == TrafficState::FAULT_SAFE) {
        logEvent(nowMs, "COMMAND_REJECTED", "Cannot inject demand in FAULT_SAFE state");
        return false;
    }
    if (static_cast<uint8_t>(movement) >= TrafficConfig::VEHICLE_GROUP_COUNT) {
        return false;
    }

    const bool wasActive = demandMgr_.getApproachState(movement).simDemandActive;
    demandMgr_.setSimulatedDemand(movement, nowMs, active);

    if (active) {
        logEvent(nowMs, wasActive ? "DEMAND_INFO" : "DEMAND_REGISTERED",
                 wasActive ? "Simulated vehicle demand already ON" : "Simulated vehicle demand ON");
        if ((movement == TrafficConfig::MovementId::SIDE_ROAD && state_ == TrafficState::SIDE_GREEN) ||
            (movement == TrafficConfig::MovementId::PROTECTED_TURN && state_ == TrafficState::TURN_GREEN) ||
            (movement == TrafficConfig::MovementId::MAIN_ROAD && state_ == TrafficState::MAIN_GREEN)) {
            sideGapTimerActive_ = false;
        }
    } else {
        logEvent(nowMs, wasActive ? "DEMAND_CLEARED" : "DEMAND_INFO",
                 wasActive ? "Simulated vehicle demand OFF" : "Simulated vehicle demand already OFF");
        if (((movement == TrafficConfig::MovementId::SIDE_ROAD && state_ == TrafficState::SIDE_GREEN) ||
             (movement == TrafficConfig::MovementId::PROTECTED_TURN && state_ == TrafficState::TURN_GREEN)) &&
            !sideGapTimerActive_) {
            sideGapTimerActive_ = true;
            sideGapStartMs_     = nowMs;
        }
    }

    refreshDemandManager(nowMs);
    return true;
}

bool TrafficController::setSimVehicleDemand(uint32_t nowMs, bool active) {
    return setSimMovementDemand(nowMs, TrafficConfig::MovementId::SIDE_ROAD, active);
}

bool TrafficController::triggerSimPedestrianChannel(uint32_t nowMs, uint8_t pedChannelIdx) {
    if (mode_ != OperatingMode::SERIAL_SIMULATION) {
        logEvent(nowMs, "COMMAND_REJECTED", "Simulation command rejected in HARDWARE_READY mode (use SIM ON)");
        return false;
    }
    if (state_ == TrafficState::FAULT_SAFE) {
        logEvent(nowMs, "COMMAND_REJECTED", "Cannot inject demand in FAULT_SAFE state");
        return false;
    }
    if (pedChannelIdx >= TrafficConfig::PED_BUTTON_COUNT) {
        return false;
    }

    if (pedCtrl_.requestPedestrian(pedChannelIdx, true)) {
        logEvent(nowMs, "PED_REQUEST_LATCHED", "Simulated pedestrian crossing request latched");
    } else {
        logEvent(nowMs, "PED_REQUEST_PENDING", "Simulated pedestrian request already latched");
    }
    return true;
}

bool TrafficController::triggerSimPedestrianRequest(uint32_t nowMs) {
    return triggerSimPedestrianChannel(nowMs, 0);
}

bool TrafficController::clearSimulatedDemand(uint32_t nowMs) {
    if (mode_ != OperatingMode::SERIAL_SIMULATION) {
        logEvent(nowMs, "COMMAND_REJECTED", "CLEAR rejected in HARDWARE_READY mode (use SIM ON)");
        return false;
    }

    const bool hadVeh =
        demandMgr_.getApproachState(TrafficConfig::MovementId::SIDE_ROAD).simDemandActive ||
        demandMgr_.getApproachState(TrafficConfig::MovementId::MAIN_ROAD).simDemandActive ||
        demandMgr_.getApproachState(TrafficConfig::MovementId::PROTECTED_TURN).simDemandActive;
    const bool hadPed = pedCtrl_.clearSimulatedRequests();

    demandMgr_.clearAllSimulatedDemand();

    if ((state_ == TrafficState::SIDE_GREEN || state_ == TrafficState::TURN_GREEN) && !sideGapTimerActive_) {
        sideGapTimerActive_ = true;
        sideGapStartMs_     = nowMs;
    }

    refreshDemandManager(nowMs);

    if (hadVeh || hadPed) {
        logEvent(nowMs, "DEMAND_CLEARED", "Cleared pending simulated demand(s)");
    } else {
        logEvent(nowMs, "DEMAND_CLEARED", "No simulated demands were pending");
    }
    return true;
}

bool TrafficController::requestEmergencyPriority(uint32_t nowMs,
                                                 TrafficConfig::MovementId movement,
                                                 const char* detectorType) {
    if (state_ == TrafficState::FAULT_SAFE) {
        return false;
    }
    if (!phaseMgr_.requestEmergencyPriority(movement, detectorType)) {
        logEvent(nowMs, "EMERGENCY_REJECTED",
                 "Emergency priority rejected (IR sensors or uncommissioned movements cannot trigger preemption)");
        return false;
    }
    logEvent(nowMs, "EMERGENCY_PRIORITY_ACTIVE", "Validated emergency priority request registered");
    return true;
}

void TrafficController::clearEmergencyPriority(uint32_t nowMs) {
    phaseMgr_.clearEmergencyPriority();
    logEvent(nowMs, "EMERGENCY_PRIORITY_CLEARED", "Emergency priority cleared");
}

void TrafficController::resetLogical(uint32_t nowMs, const char* reason) {
    faultMgr_.reset(nowMs);
    detector_.reset(nowMs);
    demandMgr_.reset(nowMs);
    pedCtrl_.reset(nowMs);
    phaseMgr_.clearEmergencyPriority();

    sideGapTimerActive_     = false;
    postPedClearanceActive_ = false;
    manualStepRequested_    = false;
    nextTarget_             = NextPhaseTarget::MAIN_ROAD;
    lastServedNonMain_      = TrafficState::STARTUP_SAFE;

    logEvent(nowMs, "LOGICAL_RESET", reason);
    const TrafficState prevState = state_;
    state_        = TrafficState::STARTUP_SAFE;
    stateStartMs_ = nowMs;
    if (transitionCb_ && prevState != TrafficState::STARTUP_SAFE) {
        transitionCb_(nowMs, prevState, TrafficState::STARTUP_SAFE, reason);
    }
}

void TrafficController::triggerFault(uint32_t nowMs, const char* reason, FaultCode code) {
    const char* safeReason = (reason && reason[0] != '\0') ? reason : "INTERNAL_CONTROL_FAULT";
    faultMgr_.recordCriticalFault(nowMs, code, safeReason);
    pedCtrl_.setAllSafeOff();
    logEvent(nowMs, "FAULT_DETECTED", safeReason);
    if (state_ != TrafficState::FAULT_SAFE) {
        const TrafficState prev = state_;
        state_        = TrafficState::FAULT_SAFE;
        stateStartMs_ = nowMs;
        if (transitionCb_) {
            transitionCb_(nowMs, prev, TrafficState::FAULT_SAFE, safeReason);
        }
    }
}

bool TrafficController::clearFaultAndRecover(uint32_t nowMs, const char* reason) {
    if (state_ != TrafficState::FAULT_SAFE && !faultMgr_.isCriticalFaultLatched()) {
        return false;
    }
    faultMgr_.clearCriticalFault(nowMs);
    // Recover through STARTUP_SAFE -> ALL_RED -> MAIN_GREEN so recovery never
    // immediately introduces a conflicting green.
    resetLogical(nowMs, reason ? reason : "OPERATOR_FAULT_RECOVERY");
    return true;
}

NextPhaseTarget TrafficController::selectNextTargetFixed(TrafficState currentGreen) const {
    if (currentGreen == TrafficState::MAIN_GREEN) {
        return NextPhaseTarget::SIDE_ROAD;
    }
    if (currentGreen == TrafficState::SIDE_GREEN) {
        if (phaseMgr_.isProtectedTurnCommissioned()) {
            return NextPhaseTarget::PROTECTED_TURN;
        }
        if (pedCtrl_.hasAnyDemand()) {
            return NextPhaseTarget::PEDESTRIAN;
        }
        return NextPhaseTarget::MAIN_ROAD;
    }
    if (currentGreen == TrafficState::TURN_GREEN) {
        if (pedCtrl_.hasAnyDemand()) {
            return NextPhaseTarget::PEDESTRIAN;
        }
        return NextPhaseTarget::MAIN_ROAD;
    }
    return NextPhaseTarget::MAIN_ROAD;
}

NextPhaseTarget TrafficController::selectNextTargetManual(TrafficState currentGreen) const {
    if (pedCtrl_.hasAnyDemand() && currentGreen != TrafficState::PED_WALK) {
        return NextPhaseTarget::PEDESTRIAN;
    }
    return selectNextTargetFixed(currentGreen);
}

NextPhaseTarget TrafficController::selectNextTargetAdaptive(TrafficConfig::MovementId currentMovement) const {
    // Check emergency priority first
    if (phaseMgr_.isEmergencyPriorityActive()) {
        const TrafficConfig::MovementId em = phaseMgr_.getEmergencyMovement();
        if (em == TrafficConfig::MovementId::MAIN_ROAD) return NextPhaseTarget::MAIN_ROAD;
        if (em == TrafficConfig::MovementId::SIDE_ROAD) return NextPhaseTarget::SIDE_ROAD;
        if (em == TrafficConfig::MovementId::PROTECTED_TURN && phaseMgr_.isProtectedTurnCommissioned()) {
            return NextPhaseTarget::PROTECTED_TURN;
        }
    }

    const uint32_t mainScore = (currentMovement == TrafficConfig::MovementId::MAIN_ROAD)
                                   ? 0
                                   : demandMgr_.calculateScore(TrafficConfig::MovementId::MAIN_ROAD);
    const uint32_t sideScore = (currentMovement == TrafficConfig::MovementId::SIDE_ROAD)
                                   ? 0
                                   : demandMgr_.calculateScore(TrafficConfig::MovementId::SIDE_ROAD);
    const uint32_t turnScore = (currentMovement == TrafficConfig::MovementId::PROTECTED_TURN ||
                                !phaseMgr_.isProtectedTurnCommissioned())
                                   ? 0
                                   : demandMgr_.calculateScore(TrafficConfig::MovementId::PROTECTED_TURN);

    // Pedestrian score: when requested, competes fairly and prevents starvation
    uint32_t pedScore = 0;
    if (pedCtrl_.hasAnyDemand()) {
        pedScore = static_cast<uint32_t>(demandMgr_.getDemandConfig().activeSensorWeight) * 2U;
        if (lastServedNonMain_ != TrafficState::PED_WALK) {
            pedScore += 5U;
        }
    }

    // Choose the highest scoring waiting target
    uint32_t bestScore = 0;
    NextPhaseTarget bestTarget = (currentMovement == TrafficConfig::MovementId::MAIN_ROAD)
                                     ? NextPhaseTarget::MAIN_ROAD
                                     : NextPhaseTarget::MAIN_ROAD;

    if (pedScore > bestScore) {
        bestScore  = pedScore;
        bestTarget = NextPhaseTarget::PEDESTRIAN;
    }
    if (turnScore > bestScore) {
        bestScore  = turnScore;
        bestTarget = NextPhaseTarget::PROTECTED_TURN;
    }
    if (sideScore > bestScore) {
        bestScore  = sideScore;
        bestTarget = NextPhaseTarget::SIDE_ROAD;
    }
    if (mainScore > bestScore) {
        bestScore  = mainScore;
        bestTarget = NextPhaseTarget::MAIN_ROAD;
    }

    if (bestScore == 0) {
        return NextPhaseTarget::MAIN_ROAD;
    }
    return bestTarget;
}

NextPhaseTarget TrafficController::selectNextTargetAfterMain() const {
    if (phaseMgr_.isEmergencyPriorityActive()) {
        const TrafficConfig::MovementId em = phaseMgr_.getEmergencyMovement();
        if (em == TrafficConfig::MovementId::SIDE_ROAD) return NextPhaseTarget::SIDE_ROAD;
        if (em == TrafficConfig::MovementId::PROTECTED_TURN && phaseMgr_.isProtectedTurnCommissioned()) {
            return NextPhaseTarget::PROTECTED_TURN;
        }
        return NextPhaseTarget::MAIN_ROAD;
    }

    if (controlMode_ == ControlMode::FIXED) {
        return selectNextTargetFixed(TrafficState::MAIN_GREEN);
    }
    if (controlMode_ == ControlMode::ADAPTIVE) {
        return selectNextTargetAdaptive(TrafficConfig::MovementId::MAIN_ROAD);
    }

    const bool pedPending  = pedCtrl_.hasAnyDemand();
    const bool sidePending = hasSideVehicleDemand();
    const bool turnPending = hasTurnVehicleDemand();

    if (pedPending && (sidePending || turnPending)) {
        if (lastServedNonMain_ == TrafficState::PED_WALK) {
            if (sidePending && (!turnPending || lastServedNonMain_ != TrafficState::SIDE_GREEN)) {
                return NextPhaseTarget::SIDE_ROAD;
            }
            if (turnPending) {
                return NextPhaseTarget::PROTECTED_TURN;
            }
            return NextPhaseTarget::SIDE_ROAD;
        }
        return NextPhaseTarget::PEDESTRIAN;
    }
    if (pedPending) {
        return NextPhaseTarget::PEDESTRIAN;
    }
    if (sidePending && turnPending) {
        if (lastServedNonMain_ == TrafficState::SIDE_GREEN) {
            return NextPhaseTarget::PROTECTED_TURN;
        }
        return NextPhaseTarget::SIDE_ROAD;
    }
    if (sidePending) {
        return NextPhaseTarget::SIDE_ROAD;
    }
    if (turnPending) {
        return NextPhaseTarget::PROTECTED_TURN;
    }
    return NextPhaseTarget::MAIN_ROAD;
}

NextPhaseTarget TrafficController::selectNextTargetAfterSide() const {
    if (controlMode_ == ControlMode::FIXED) {
        return selectNextTargetFixed(TrafficState::SIDE_GREEN);
    }
    if (controlMode_ == ControlMode::ADAPTIVE) {
        return selectNextTargetAdaptive(TrafficConfig::MovementId::SIDE_ROAD);
    }
    if (pedCtrl_.hasAnyDemand()) {
        return NextPhaseTarget::PEDESTRIAN;
    }
    if (hasTurnVehicleDemand() && !hasMainVehicleDemand()) {
        return NextPhaseTarget::PROTECTED_TURN;
    }
    return NextPhaseTarget::MAIN_ROAD;
}

NextPhaseTarget TrafficController::selectNextTargetAfterTurn() const {
    if (controlMode_ == ControlMode::FIXED) {
        return selectNextTargetFixed(TrafficState::TURN_GREEN);
    }
    if (controlMode_ == ControlMode::ADAPTIVE) {
        return selectNextTargetAdaptive(TrafficConfig::MovementId::PROTECTED_TURN);
    }
    if (pedCtrl_.hasAnyDemand()) {
        return NextPhaseTarget::PEDESTRIAN;
    }
    return NextPhaseTarget::MAIN_ROAD;
}

void TrafficController::update(uint32_t nowMs) {
    refreshDemandManager(nowMs);

    // Verify output invariants before running state progression
    const SignalOutputs currentOut = getOutputs();
    const char* violation = nullptr;
    if (!verifySafetyInvariants(state_, currentOut, &violation)) {
        triggerFault(nowMs,
                     violation ? violation : "SAFETY_INVARIANT_VIOLATION",
                     FaultCode::ILLEGAL_SIGNAL_COMBINATION);
        return;
    }

    if (state_ == TrafficState::FAULT_SAFE) {
        return;
    }

    // Protected Turn uncommissioned guard
    if ((state_ == TrafficState::TURN_GREEN || state_ == TrafficState::TURN_YELLOW) &&
        !phaseMgr_.isProtectedTurnCommissioned()) {
        triggerFault(nowMs, "UNCOMMISSIONED_PROTECTED_TURN_ACTIVE", FaultCode::UNCOMMISSIONED_TURN_CONFLICT);
        return;
    }

    // Rollover-safe unsigned subtraction
    const uint32_t elapsed = nowMs - stateStartMs_;

    // Timing fault watchdog: no normal non-resting phase may exceed MAX_ALLOWED_INTERVAL_MS * 2
    if (state_ != TrafficState::MAIN_GREEN &&
        controlMode_ != ControlMode::MANUAL &&
        elapsed > (TrafficConfig::MAX_ALLOWED_INTERVAL_MS * 2U)) {
        triggerFault(nowMs, "PHASE_TIMING_WATCHDOG_EXCEEDED", FaultCode::CONTROLLER_TIMING_FAULT);
        return;
    }

    switch (state_) {
        case TrafficState::STARTUP_SAFE: {
            if (elapsed >= timings_.startupSafeMs) {
                nextTarget_ = NextPhaseTarget::MAIN_ROAD;
                pedCtrl_.setAllDontWalk();
                transitionTo(nowMs, TrafficState::ALL_RED, "STARTUP_CLEARANCE_COMPLETE");
            }
            break;
        }

        case TrafficState::ALL_RED: {
            const uint32_t requiredClearanceMs = postPedClearanceActive_
                                                     ? timings_.pedClearanceMs
                                                     : timings_.allRedMs;
            if (elapsed >= requiredClearanceMs) {
                if (postPedClearanceActive_) {
                    postPedClearanceActive_ = false;
                    pedCtrl_.endClearancePhase();
                }

                if (nextTarget_ == NextPhaseTarget::SIDE_ROAD) {
                    sideGapTimerActive_ = !hasActivePresenceForCurrentPhase(TrafficConfig::MovementId::SIDE_ROAD);
                    sideGapStartMs_     = nowMs;
                    demandMgr_.notifyPhaseServed(nowMs, TrafficConfig::MovementId::SIDE_ROAD);
                    transitionTo(nowMs, TrafficState::SIDE_GREEN, "ALL_RED_CLEARANCE_COMPLETE");
                } else if (nextTarget_ == NextPhaseTarget::PROTECTED_TURN &&
                           phaseMgr_.isProtectedTurnCommissioned()) {
                    sideGapTimerActive_ = !hasActivePresenceForCurrentPhase(TrafficConfig::MovementId::PROTECTED_TURN);
                    sideGapStartMs_     = nowMs;
                    demandMgr_.notifyPhaseServed(nowMs, TrafficConfig::MovementId::PROTECTED_TURN);
                    transitionTo(nowMs, TrafficState::TURN_GREEN, "ALL_RED_CLEARANCE_COMPLETE");
                } else if (nextTarget_ == NextPhaseTarget::PEDESTRIAN) {
                    pedCtrl_.beginWalkPhase();
                    transitionTo(nowMs, TrafficState::PED_WALK, "ALL_RED_CLEARANCE_COMPLETE");
                } else {
                    sideGapTimerActive_ = false;
                    demandMgr_.notifyPhaseServed(nowMs, TrafficConfig::MovementId::MAIN_ROAD);
                    transitionTo(nowMs, TrafficState::MAIN_GREEN, "ALL_RED_CLEARANCE_COMPLETE");
                }
            }
            break;
        }

        case TrafficState::MAIN_GREEN: {
            if (controlMode_ == ControlMode::MANUAL) {
                if (manualStepRequested_ && elapsed >= timings_.mainMinGreenMs) {
                    manualStepRequested_ = false;
                    nextTarget_          = selectNextTargetManual(TrafficState::MAIN_GREEN);
                    transitionTo(nowMs, TrafficState::MAIN_YELLOW, "MANUAL_STEP_ADVANCE");
                }
                break;
            }

            if (elapsed >= timings_.mainMinGreenMs) {
                const NextPhaseTarget target = selectNextTargetAfterMain();
                if (target != NextPhaseTarget::MAIN_ROAD) {
                    // In ACTUATED or ADAPTIVE mode, if Main Road has active vehicle presence,
                    // extend Main Green up to mainMaxGreenMs before yielding.
                    const bool mainPresence = hasActivePresenceForCurrentPhase(TrafficConfig::MovementId::MAIN_ROAD);
                    const bool canExtendMain =
                        (controlMode_ == ControlMode::ACTUATED || controlMode_ == ControlMode::ADAPTIVE) &&
                        mainPresence &&
                        !phaseMgr_.isEmergencyPriorityActive() &&
                        (elapsed < timings_.mainMaxGreenMs);

                    if (!canExtendMain) {
                        detector_.clearLatchedDemandForMovement(TrafficConfig::MovementId::MAIN_ROAD);
                        nextTarget_ = target;
                        const char* reason = (target == NextPhaseTarget::PEDESTRIAN)
                                                 ? "PEDESTRIAN_DEMAND"
                                                 : ((target == NextPhaseTarget::PROTECTED_TURN)
                                                        ? "PROTECTED_TURN_DEMAND"
                                                        : "SIDE_ROAD_DEMAND");
                        transitionTo(nowMs, TrafficState::MAIN_YELLOW, reason);
                    }
                }
            }
            break;
        }

        case TrafficState::MAIN_YELLOW: {
            if (elapsed >= timings_.mainYellowMs) {
                if (controlMode_ != ControlMode::MANUAL && controlMode_ != ControlMode::FIXED) {
                    nextTarget_ = selectNextTargetAfterMain();
                }
                transitionTo(nowMs, TrafficState::ALL_RED, "MAIN_YELLOW_COMPLETE");
            }
            break;
        }

        case TrafficState::SIDE_GREEN: {
            if (controlMode_ == ControlMode::MANUAL) {
                if (manualStepRequested_ && elapsed >= timings_.sideMinGreenMs) {
                    manualStepRequested_ = false;
                    detector_.clearLatchedDemandForMovement(TrafficConfig::MovementId::SIDE_ROAD);
                    lastServedNonMain_ = TrafficState::SIDE_GREEN;
                    nextTarget_        = selectNextTargetManual(TrafficState::SIDE_GREEN);
                    transitionTo(nowMs, TrafficState::SIDE_YELLOW, "MANUAL_STEP_ADVANCE");
                }
                break;
            }

            if (controlMode_ == ControlMode::FIXED) {
                if (elapsed >= timings_.sideMinGreenMs) {
                    detector_.clearLatchedDemandForMovement(TrafficConfig::MovementId::SIDE_ROAD);
                    lastServedNonMain_ = TrafficState::SIDE_GREEN;
                    nextTarget_        = selectNextTargetAfterSide();
                    transitionTo(nowMs, TrafficState::SIDE_YELLOW, "FIXED_SIDE_GREEN_COMPLETE");
                }
                break;
            }

            const bool activePresence = hasActivePresenceForCurrentPhase(TrafficConfig::MovementId::SIDE_ROAD);
            if (activePresence) {
                sideGapTimerActive_ = false;
            } else if (!sideGapTimerActive_) {
                sideGapTimerActive_ = true;
                sideGapStartMs_     = nowMs;
            }

            const uint32_t gapElapsed = sideGapTimerActive_ ? (nowMs - sideGapStartMs_) : 0;

            if (elapsed >= timings_.sideMaxGreenMs) {
                detector_.clearLatchedDemandForMovement(TrafficConfig::MovementId::SIDE_ROAD);
                lastServedNonMain_ = TrafficState::SIDE_GREEN;
                nextTarget_        = selectNextTargetAfterSide();
                logEvent(nowMs, "SERVICE_COMPLETED", "Side-road service ended (MAX_GREEN reached)");
                transitionTo(nowMs, TrafficState::SIDE_YELLOW, "SIDE_MAX_GREEN_ELAPSED");
            } else if (elapsed >= timings_.sideMinGreenMs &&
                       sideGapTimerActive_ &&
                       gapElapsed >= timings_.sideClearGapMs) {
                detector_.clearLatchedDemandForMovement(TrafficConfig::MovementId::SIDE_ROAD);
                lastServedNonMain_ = TrafficState::SIDE_GREEN;
                nextTarget_        = selectNextTargetAfterSide();
                logEvent(nowMs, "SERVICE_COMPLETED", "Side-road service ended (demand cleared + gap elapsed)");
                transitionTo(nowMs, TrafficState::SIDE_YELLOW, "SIDE_DEMAND_GAP_CLEARED");
            }
            break;
        }

        case TrafficState::SIDE_YELLOW: {
            if (elapsed >= timings_.sideYellowMs) {
                if (controlMode_ == ControlMode::AUTO) {
                    nextTarget_ = NextPhaseTarget::MAIN_ROAD;
                }
                transitionTo(nowMs, TrafficState::ALL_RED, "SIDE_YELLOW_COMPLETE");
            }
            break;
        }

        case TrafficState::TURN_GREEN: {
            if (controlMode_ == ControlMode::MANUAL) {
                if (manualStepRequested_ && elapsed >= timings_.turnMinGreenMs) {
                    manualStepRequested_ = false;
                    detector_.clearLatchedDemandForMovement(TrafficConfig::MovementId::PROTECTED_TURN);
                    lastServedNonMain_ = TrafficState::TURN_GREEN;
                    nextTarget_        = selectNextTargetManual(TrafficState::TURN_GREEN);
                    transitionTo(nowMs, TrafficState::TURN_YELLOW, "MANUAL_STEP_ADVANCE");
                }
                break;
            }

            if (controlMode_ == ControlMode::FIXED) {
                if (elapsed >= timings_.turnMinGreenMs) {
                    detector_.clearLatchedDemandForMovement(TrafficConfig::MovementId::PROTECTED_TURN);
                    lastServedNonMain_ = TrafficState::TURN_GREEN;
                    nextTarget_        = selectNextTargetAfterTurn();
                    transitionTo(nowMs, TrafficState::TURN_YELLOW, "FIXED_TURN_GREEN_COMPLETE");
                }
                break;
            }

            const bool activePresence = hasActivePresenceForCurrentPhase(TrafficConfig::MovementId::PROTECTED_TURN);
            if (activePresence) {
                sideGapTimerActive_ = false;
            } else if (!sideGapTimerActive_) {
                sideGapTimerActive_ = true;
                sideGapStartMs_     = nowMs;
            }

            const uint32_t gapElapsed = sideGapTimerActive_ ? (nowMs - sideGapStartMs_) : 0;

            if (elapsed >= timings_.turnMaxGreenMs) {
                detector_.clearLatchedDemandForMovement(TrafficConfig::MovementId::PROTECTED_TURN);
                lastServedNonMain_ = TrafficState::TURN_GREEN;
                nextTarget_        = selectNextTargetAfterTurn();
                logEvent(nowMs, "SERVICE_COMPLETED", "Protected-turn service ended (MAX_GREEN reached)");
                transitionTo(nowMs, TrafficState::TURN_YELLOW, "TURN_MAX_GREEN_ELAPSED");
            } else if (elapsed >= timings_.turnMinGreenMs &&
                       sideGapTimerActive_ &&
                       gapElapsed >= timings_.turnClearGapMs) {
                detector_.clearLatchedDemandForMovement(TrafficConfig::MovementId::PROTECTED_TURN);
                lastServedNonMain_ = TrafficState::TURN_GREEN;
                nextTarget_        = selectNextTargetAfterTurn();
                logEvent(nowMs, "SERVICE_COMPLETED", "Protected-turn service ended (demand cleared + gap elapsed)");
                transitionTo(nowMs, TrafficState::TURN_YELLOW, "TURN_DEMAND_GAP_CLEARED");
            }
            break;
        }

        case TrafficState::TURN_YELLOW: {
            if (elapsed >= timings_.turnYellowMs) {
                if (controlMode_ == ControlMode::AUTO) {
                    nextTarget_ = NextPhaseTarget::MAIN_ROAD;
                }
                transitionTo(nowMs, TrafficState::ALL_RED, "TURN_YELLOW_COMPLETE");
            }
            break;
        }

        case TrafficState::PED_WALK: {
            const bool walkDone = (controlMode_ == ControlMode::MANUAL)
                                      ? (manualStepRequested_ && elapsed >= timings_.pedWalkMs)
                                      : (elapsed >= timings_.pedWalkMs);
            if (walkDone) {
                manualStepRequested_    = false;
                pedCtrl_.beginClearancePhase();
                postPedClearanceActive_ = true;
                lastServedNonMain_      = TrafficState::PED_WALK;
                nextTarget_             = (controlMode_ == ControlMode::ADAPTIVE)
                                              ? selectNextTargetAdaptive(TrafficConfig::MovementId::PED_CROSSING_1)
                                              : NextPhaseTarget::MAIN_ROAD;
                logEvent(nowMs, "SERVICE_COMPLETED", "Pedestrian WALK phase completed; entering clearance");
                transitionTo(nowMs, TrafficState::ALL_RED, "PED_WALK_COMPLETE");
            }
            break;
        }

        case TrafficState::FAULT_SAFE:
        default:
            break;
    }
}

bool TrafficController::isValidTransition(TrafficState from, TrafficState to) const {
    if (to == TrafficState::FAULT_SAFE || to == TrafficState::STARTUP_SAFE) {
        return true;
    }

    switch (from) {
        case TrafficState::STARTUP_SAFE:
            return (to == TrafficState::ALL_RED);
        case TrafficState::ALL_RED:
            if (to == TrafficState::TURN_GREEN && !phaseMgr_.isProtectedTurnCommissioned()) {
                return false;
            }
            return (to == TrafficState::MAIN_GREEN ||
                    to == TrafficState::SIDE_GREEN ||
                    to == TrafficState::TURN_GREEN ||
                    to == TrafficState::PED_WALK);
        case TrafficState::MAIN_GREEN:
            return (to == TrafficState::MAIN_YELLOW);
        case TrafficState::MAIN_YELLOW:
            return (to == TrafficState::ALL_RED);
        case TrafficState::SIDE_GREEN:
            return (to == TrafficState::SIDE_YELLOW);
        case TrafficState::SIDE_YELLOW:
            return (to == TrafficState::ALL_RED);
        case TrafficState::TURN_GREEN:
            return (to == TrafficState::TURN_YELLOW);
        case TrafficState::TURN_YELLOW:
            return (to == TrafficState::ALL_RED);
        case TrafficState::PED_WALK:
            return (to == TrafficState::ALL_RED);
        case TrafficState::FAULT_SAFE:
            return false;
        default:
            return false;
    }
}

void TrafficController::transitionTo(uint32_t nowMs, TrafficState newState, const char* reason) {
    if (!isValidTransition(state_, newState)) {
        triggerFault(nowMs, "INVALID_PHASE_TRANSITION", FaultCode::UNEXPECTED_STATE_TRANSITION);
        return;
    }

    // Check explicit movement conflict matrix before entering any active phase
    bool activeMovements[static_cast<size_t>(TrafficConfig::MovementId::COUNT)] = {
        false, false, false, false, false
    };
    if (newState == TrafficState::MAIN_GREEN) {
        if (!phaseMgr_.isMovementPermitted(TrafficConfig::MovementId::MAIN_ROAD, activeMovements)) {
            triggerFault(nowMs, "MAIN_MOVEMENT_CONFLICT", FaultCode::ILLEGAL_SIGNAL_COMBINATION);
            return;
        }
    } else if (newState == TrafficState::SIDE_GREEN) {
        if (!phaseMgr_.isMovementPermitted(TrafficConfig::MovementId::SIDE_ROAD, activeMovements)) {
            triggerFault(nowMs, "SIDE_MOVEMENT_CONFLICT", FaultCode::ILLEGAL_SIGNAL_COMBINATION);
            return;
        }
    } else if (newState == TrafficState::TURN_GREEN) {
        if (!phaseMgr_.isMovementPermitted(TrafficConfig::MovementId::PROTECTED_TURN, activeMovements)) {
            triggerFault(nowMs, "PROTECTED_TURN_NOT_PERMITTED", FaultCode::UNCOMMISSIONED_TURN_CONFLICT);
            return;
        }
    }

    const SignalOutputs candidateOut = deriveOutputsForState(newState);
    const char* violation = nullptr;
    if (!verifySafetyInvariants(newState, candidateOut, &violation)) {
        triggerFault(nowMs,
                     violation ? violation : "CANDIDATE_OUTPUT_INVARIANT_VIOLATION",
                     FaultCode::ILLEGAL_SIGNAL_COMBINATION);
        return;
    }

    const TrafficState oldState = state_;
    state_        = newState;
    stateStartMs_ = nowMs;

    if (transitionCb_) {
        transitionCb_(nowMs, oldState, newState, reason);
    }
}

void TrafficController::logEvent(uint32_t nowMs, const char* eventType, const char* detail) {
    if (eventLogCb_) {
        eventLogCb_(nowMs, eventType, detail);
    }
}

SignalOutputs TrafficController::getOutputs() const {
    SignalOutputs out = deriveOutputsForState(state_);
    if (state_ == TrafficState::PED_WALK) {
        // Reflect individual pedestrian group WALK states while maintaining RED/GREEN mutual exclusion
        const bool p1Walk = pedCtrl_.isWalkActive(0);
        const bool p2Walk = pedCtrl_.isWalkActive(1);
        if (p1Walk || p2Walk) {
            out.pedWalk      = p1Walk;
            out.pedDontWalk  = !p1Walk;
            out.ped2Walk     = p2Walk;
            out.ped2DontWalk = !p2Walk;
        }
    }
    return out;
}

SignalOutputs TrafficController::deriveOutputsForState(TrafficState state) {
    SignalOutputs out = {
        false, false, false, false, false, false, false, false,
        false, false, false, false, false
    };

    switch (state) {
        case TrafficState::STARTUP_SAFE:
            break;

        case TrafficState::ALL_RED:
            out.mainRed      = true;
            out.sideRed      = true;
            out.turnRed      = true;
            out.pedDontWalk  = true;
            out.ped2DontWalk = true;
            break;

        case TrafficState::MAIN_GREEN:
            out.mainGreen    = true;
            out.sideRed      = true;
            out.turnRed      = true;
            out.pedDontWalk  = true;
            out.ped2DontWalk = true;
            break;

        case TrafficState::MAIN_YELLOW:
            out.mainYellow   = true;
            out.sideRed      = true;
            out.turnRed      = true;
            out.pedDontWalk  = true;
            out.ped2DontWalk = true;
            break;

        case TrafficState::SIDE_GREEN:
            out.mainRed      = true;
            out.sideGreen    = true;
            out.turnRed      = true;
            out.pedDontWalk  = true;
            out.ped2DontWalk = true;
            break;

        case TrafficState::SIDE_YELLOW:
            out.mainRed      = true;
            out.sideYellow   = true;
            out.turnRed      = true;
            out.pedDontWalk  = true;
            out.ped2DontWalk = true;
            break;

        case TrafficState::TURN_GREEN:
            out.mainRed      = true;
            out.sideRed      = true;
            out.turnGreen    = true;
            out.pedDontWalk  = true;
            out.ped2DontWalk = true;
            break;

        case TrafficState::TURN_YELLOW:
            out.mainRed      = true;
            out.sideRed      = true;
            out.turnYellow   = true;
            out.pedDontWalk  = true;
            out.ped2DontWalk = true;
            break;

        case TrafficState::PED_WALK:
            out.mainRed      = true;
            out.sideRed      = true;
            out.turnRed      = true;
            out.pedWalk      = true;
            out.ped2Walk     = true;
            break;

        case TrafficState::FAULT_SAFE:
        default:
            break;
    }

    return out;
}

bool TrafficController::verifySafetyInvariants(TrafficState state,
                                               const SignalOutputs& out,
                                               const char** violationReason) {
    // Invariant 1: No two vehicle movements may display GREEN (or YELLOW) simultaneously.
    const uint8_t activeVehicleGreens =
        (out.mainGreen ? 1 : 0) + (out.sideGreen ? 1 : 0) + (out.turnGreen ? 1 : 0);
    if (activeVehicleGreens > 1) {
        if (violationReason) *violationReason = "CONFLICTING_VEHICLE_GREENS";
        return false;
    }

    const bool mainMoving = out.mainGreen || out.mainYellow;
    const bool sideMoving = out.sideGreen || out.sideYellow;
    const bool turnMoving = out.turnGreen || out.turnYellow;
    if ((mainMoving && sideMoving) || (mainMoving && turnMoving) || (sideMoving && turnMoving)) {
        if (violationReason) *violationReason = "CONFLICTING_VEHICLE_MOVEMENTS";
        return false;
    }

    // Invariant 2: Pedestrian WALK must never be active concurrently with any vehicle GREEN or YELLOW.
    if ((out.pedWalk || out.ped2Walk) && (mainMoving || sideMoving || turnMoving)) {
        if (violationReason) *violationReason = "WALK_CONCURRENT_WITH_VEHICLE_GREEN";
        return false;
    }

    // Invariant 3: All vehicle approaches must be logically RED during pedestrian WALK,
    // and pedestrian RED + GREEN must never be simultaneous.
    if (state == TrafficState::PED_WALK || out.pedWalk || out.ped2Walk) {
        if (!out.mainRed || !out.sideRed || mainMoving || sideMoving || turnMoving) {
            if (violationReason) *violationReason = "VEHICLES_NOT_RED_DURING_PED_WALK";
            return false;
        }
        if (out.pedWalk && out.pedDontWalk) {
            if (violationReason) *violationReason = "PED_WALK_AND_DONT_WALK_CONCURRENT";
            return false;
        }
        if (out.ped2Walk && out.ped2DontWalk) {
            if (violationReason) *violationReason = "PED2_WALK_AND_DONT_WALK_CONCURRENT";
            return false;
        }
    }

    // Invariant 4: No simultaneous RED and GREEN (or YELLOW+GREEN or YELLOW+RED) on any signal group.
    if ((out.mainYellow && out.mainGreen) || (out.mainYellow && out.mainRed)) {
        if (violationReason) *violationReason = "MAIN_APPROACH_CONFLICTING_ASPECTS";
        return false;
    }
    if ((out.sideYellow && out.sideGreen) || (out.sideYellow && out.sideRed)) {
        if (violationReason) *violationReason = "SIDE_APPROACH_CONFLICTING_ASPECTS";
        return false;
    }
    if ((out.turnYellow && out.turnGreen) || (out.turnYellow && out.turnRed)) {
        if (violationReason) *violationReason = "TURN_APPROACH_CONFLICTING_ASPECTS";
        return false;
    }
    if (out.mainGreen && out.mainRed) {
        if (violationReason) *violationReason = "MAIN_GREEN_AND_RED_CONCURRENT";
        return false;
    }
    if (out.sideGreen && out.sideRed) {
        if (violationReason) *violationReason = "SIDE_GREEN_AND_RED_CONCURRENT";
        return false;
    }
    if (out.turnGreen && out.turnRed) {
        if (violationReason) *violationReason = "TURN_GREEN_AND_RED_CONCURRENT";
        return false;
    }
    if (out.pedWalk && out.pedDontWalk) {
        if (violationReason) *violationReason = "PED_WALK_AND_DONT_WALK_CONCURRENT";
        return false;
    }
    if (out.ped2Walk && out.ped2DontWalk) {
        if (violationReason) *violationReason = "PED2_WALK_AND_DONT_WALK_CONCURRENT";
        return false;
    }

    // Invariant 5: In FAULT_SAFE and STARTUP_SAFE, all logical outputs must be inactive (false).
    if (state == TrafficState::FAULT_SAFE || state == TrafficState::STARTUP_SAFE) {
        if (out.mainGreen || out.mainYellow || out.mainRed ||
            out.sideGreen || out.sideYellow || out.sideRed ||
            out.turnGreen || out.turnYellow || out.turnRed ||
            out.pedWalk   || out.pedDontWalk ||
            out.ped2Walk  || out.ped2DontWalk) {
            if (violationReason) *violationReason = "OUTPUTS_ACTIVE_IN_SAFE_OFF_STATE";
            return false;
        }
    }

    return true;
}

const char* TrafficController::stateToString(TrafficState s) {
    switch (s) {
        case TrafficState::STARTUP_SAFE: return "STARTUP_SAFE";
        case TrafficState::ALL_RED:      return "ALL_RED";
        case TrafficState::MAIN_GREEN:   return "MAIN_GREEN";
        case TrafficState::MAIN_YELLOW:  return "MAIN_YELLOW";
        case TrafficState::SIDE_GREEN:   return "SIDE_GREEN";
        case TrafficState::SIDE_YELLOW:  return "SIDE_YELLOW";
        case TrafficState::TURN_GREEN:   return "TURN_GREEN";
        case TrafficState::TURN_YELLOW:  return "TURN_YELLOW";
        case TrafficState::PED_WALK:     return "PED_WALK";
        case TrafficState::FAULT_SAFE:   return "FAULT_SAFE";
        default:                         return "UNKNOWN";
    }
}

const char* TrafficController::modeToString(OperatingMode m) {
    switch (m) {
        case OperatingMode::HARDWARE_READY:    return "HARDWARE_READY";
        case OperatingMode::SERIAL_SIMULATION: return "SERIAL_SIMULATION";
        default:                               return "UNKNOWN";
    }
}

const char* TrafficController::controlModeToString(ControlMode m) {
    return TrafficPhaseManager::controlModeToString(m);
}

const char* TrafficController::targetToString(NextPhaseTarget t) {
    switch (t) {
        case NextPhaseTarget::MAIN_ROAD:      return "MAIN_ROAD";
        case NextPhaseTarget::SIDE_ROAD:      return "SIDE_ROAD";
        case NextPhaseTarget::PEDESTRIAN:     return "PEDESTRIAN";
        case NextPhaseTarget::PROTECTED_TURN: return "PROTECTED_TURN";
        default:                              return "UNKNOWN";
    }
}
