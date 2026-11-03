#include "TrafficController.h"

TrafficController::TrafficController()
    : timings_(TrafficConfig::DEFAULT_TIMING_CONFIG),
      state_(TrafficState::STARTUP_SAFE),
      lastServedNonMain_(TrafficState::STARTUP_SAFE),
      mode_(OperatingMode::HARDWARE_READY),
      nextTarget_(NextPhaseTarget::MAIN_ROAD),
      stateStartMs_(0),
      sideGapStartMs_(0),
      sideGapTimerActive_(false),
      pedRequestLatched_(false),
      pedRequestFromSim_(false),
      simVehiclePresent_(false),
      hwVehicleLatched_(false),
      hwBtnLastRawPressed_(false),
      hwBtnStablePressed_(false),
      hwBtnAwaitRelease_(false),
      hwBtnLastChangeMs_(0),
      hwIrLastRawDetected_(false),
      hwIrStableDetected_(false),
      hwIrStuckFault_(false),
      hwIrLastChangeMs_(0),
      hwIrActiveStartMs_(0),
      physicalIrValidated_(TrafficConfig::PHYSICAL_IR_VALIDATED),
      faultReason_("NONE"),
      transitionCb_(nullptr),
      eventLogCb_(nullptr) {}

void TrafficController::begin(uint32_t nowMs) {
    validateAndSanitizeTimings(timings_);

    state_              = TrafficState::STARTUP_SAFE;
    lastServedNonMain_  = TrafficState::STARTUP_SAFE;
    mode_               = OperatingMode::HARDWARE_READY;
    nextTarget_         = NextPhaseTarget::MAIN_ROAD;
    stateStartMs_       = nowMs;
    sideGapStartMs_     = nowMs;
    sideGapTimerActive_ = false;

    pedRequestLatched_  = false;
    pedRequestFromSim_  = false;
    simVehiclePresent_  = false;
    hwVehicleLatched_   = false;

    hwBtnLastRawPressed_ = false;
    hwBtnStablePressed_  = false;
    hwBtnAwaitRelease_   = false;
    hwBtnLastChangeMs_   = nowMs;

    hwIrLastRawDetected_ = false;
    hwIrStableDetected_  = false;
    hwIrStuckFault_      = false;
    hwIrLastChangeMs_    = nowMs;
    hwIrActiveStartMs_   = nowMs;

    faultReason_        = "NONE";
}

bool TrafficController::validateAndSanitizeTimings(TrafficConfig::TimingConfig& cfg) {
    bool allValid = true;
    const auto checkRange = [&](uint32_t& val, uint32_t fallback) {
        if (val < TrafficConfig::MIN_ALLOWED_INTERVAL_MS || val > TrafficConfig::MAX_ALLOWED_INTERVAL_MS) {
            val = fallback;
            allValid = false;
        }
    };

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

    return allValid;
}

bool TrafficController::setTimingConfig(uint32_t nowMs, const TrafficConfig::TimingConfig& cfg) {
    TrafficConfig::TimingConfig sanitized = cfg;
    const bool valid = validateAndSanitizeTimings(sanitized);
    timings_ = sanitized;
    if (!valid) {
        logEvent(nowMs, "TIMING_FALLBACK",
                 "Invalid timing configuration detected; fell back to documented safe defaults");
    } else {
        logEvent(nowMs, "TIMING_UPDATED", "Validated timing configuration applied");
    }
    return valid;
}

bool TrafficController::hasSideVehicleDemand() const {
    if (mode_ == OperatingMode::SERIAL_SIMULATION) {
        return simVehiclePresent_;
    }
    if (!physicalIrValidated_ || hwIrStuckFault_) {
        return false;
    }
    return hwVehicleLatched_ || hwIrStableDetected_;
}

void TrafficController::processHardwareAdapter(uint32_t nowMs, const HardwareInputs& inputs) {
    if (!inputs.inputsValid) {
        // Disconnected or invalid hardware adapter frame: clear unlatched raw states safely
        hwIrLastRawDetected_ = false;
        hwIrStableDetected_  = false;
        hwIrLastChangeMs_    = nowMs;
        hwBtnLastRawPressed_ = false;
        hwBtnStablePressed_  = false;
        hwBtnLastChangeMs_   = nowMs;
        return;
    }
    processPhysicalInputs(nowMs, inputs.rawPedButtonPinHigh, inputs.rawIrSensorPinHigh);
}

void TrafficController::processPhysicalInputs(uint32_t nowMs, bool rawPedButtonPinHigh, bool rawIrSensorPinHigh) {
    if (mode_ != OperatingMode::HARDWARE_READY || state_ == TrafficState::FAULT_SAFE) {
        return;
    }

    // 1. Pedestrian Button Processing (GPIO 32, INPUT_PULLUP, active LOW)
    const bool rawPressed = TrafficConfig::PED_BTN_ACTIVE_LOW ? !rawPedButtonPinHigh : rawPedButtonPinHigh;
    if (rawPressed != hwBtnLastRawPressed_) {
        hwBtnLastRawPressed_ = rawPressed;
        hwBtnLastChangeMs_   = nowMs;
    }

    if ((nowMs - hwBtnLastChangeMs_) >= TrafficConfig::PED_DEBOUNCE_MS) {
        if (rawPressed != hwBtnStablePressed_) {
            hwBtnStablePressed_ = rawPressed;
            if (hwBtnStablePressed_ && !hwBtnAwaitRelease_) {
                hwBtnAwaitRelease_ = true;
                if (!pedRequestLatched_) {
                    pedRequestLatched_ = true;
                    pedRequestFromSim_ = false;
                    logEvent(nowMs, "PED_REQUEST_LATCHED", "Physical pedestrian button press (debounced)");
                } else {
                    logEvent(nowMs, "PED_REQUEST_PENDING", "Physical press ignored; request already latched");
                }
            } else if (!hwBtnStablePressed_) {
                // Button released stably; allow next press
                hwBtnAwaitRelease_ = false;
            }
        }
    }

    // 2. Side-Road IR Sensor Processing (GPIO 33, INPUT_PULLUP, configurable polarity)
    // Only interpret physical IR sensor readings as demand when hardware is validated
    if (!physicalIrValidated_) {
        hwIrLastRawDetected_ = false;
        hwIrStableDetected_  = false;
        hwVehicleLatched_    = false;
        return;
    }

    const bool rawDetected = TrafficConfig::IR_SNS_ACTIVE_LOW ? !rawIrSensorPinHigh : rawIrSensorPinHigh;
    if (rawDetected != hwIrLastRawDetected_) {
        hwIrLastRawDetected_ = rawDetected;
        hwIrLastChangeMs_   = nowMs;
    }

    if ((nowMs - hwIrLastChangeMs_) >= TrafficConfig::IR_STABILITY_MS) {
        if (rawDetected != hwIrStableDetected_) {
            hwIrStableDetected_ = rawDetected;
            if (hwIrStableDetected_) {
                hwIrActiveStartMs_ = nowMs;
                hwIrStuckFault_    = false;
                if (!hwVehicleLatched_) {
                    hwVehicleLatched_ = true;
                    logEvent(nowMs, "DEMAND_REGISTERED", "Physical side-road IR vehicle detected");
                }
            } else {
                if (hwIrStuckFault_) {
                    hwIrStuckFault_ = false;
                    logEvent(nowMs, "SENSOR_RECOVERED", "Physical side-road IR sensor cleared after stuck state");
                } else {
                    logEvent(nowMs, "SENSOR_CLEARED", "Physical side-road IR vehicle stream ended");
                }
            }
        } else if (hwIrStableDetected_ && !hwIrStuckFault_) {
            // Supervision: if physical sensor is shorted/stuck active beyond MAX_INPUT_STUCK_MS,
            // flag sensor fault and suppress continuous false demand so side road cannot monopolize cycle.
            if ((nowMs - hwIrActiveStartMs_) >= TrafficConfig::MAX_INPUT_STUCK_MS) {
                hwIrStuckFault_   = true;
                hwVehicleLatched_ = false;
                logEvent(nowMs, "SENSOR_FAULT_SUSPECTED",
                         "Side-road IR sensor active continuously > MAX_INPUT_STUCK_MS; suppressing stuck input");
            }
        }
    }
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

        const bool hadSimVehicle = simVehiclePresent_;
        const bool hadSimPed     = pedRequestLatched_ && pedRequestFromSim_;

        // Discard simulation-only demands immediately
        simVehiclePresent_ = false;
        if (pedRequestFromSim_) {
            pedRequestLatched_ = false;
            pedRequestFromSim_ = false;
        }

        // Cleanly initialize hardware input state
        hwVehicleLatched_    = false;
        hwBtnLastRawPressed_ = false;
        hwBtnStablePressed_  = false;
        hwBtnAwaitRelease_   = false;
        hwBtnLastChangeMs_   = nowMs;
        hwIrLastRawDetected_ = false;
        hwIrStableDetected_  = false;
        hwIrStuckFault_      = false;
        hwIrLastChangeMs_    = nowMs;
        hwIrActiveStartMs_   = nowMs;

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

        // Safeguard: If an active non-main phase cannot be safely continued or was driven by sim,
        // terminate gracefully via clearance sequence -> ALL_RED -> MAIN_GREEN.
        if (state_ == TrafficState::SIDE_GREEN) {
            nextTarget_ = NextPhaseTarget::MAIN_ROAD;
            transitionTo(nowMs, TrafficState::SIDE_YELLOW, "SIM_OFF_SAFE_RECOVERY");
        } else if (state_ == TrafficState::PED_WALK) {
            nextTarget_ = NextPhaseTarget::MAIN_ROAD;
            transitionTo(nowMs, TrafficState::ALL_RED, "SIM_OFF_SAFE_RECOVERY");
        } else if (state_ == TrafficState::MAIN_YELLOW || state_ == TrafficState::ALL_RED) {
            nextTarget_ = NextPhaseTarget::MAIN_ROAD;
        }

        return true;
    }
}

bool TrafficController::setSimVehicleDemand(uint32_t nowMs, bool active) {
    if (mode_ != OperatingMode::SERIAL_SIMULATION) {
        logEvent(nowMs, "COMMAND_REJECTED", "Simulation command rejected in HARDWARE_READY mode (use SIM ON)");
        return false;
    }
    if (state_ == TrafficState::FAULT_SAFE) {
        logEvent(nowMs, "COMMAND_REJECTED", "Cannot inject demand in FAULT_SAFE state");
        return false;
    }

    if (active) {
        if (!simVehiclePresent_) {
            simVehiclePresent_ = true;
            logEvent(nowMs, "DEMAND_REGISTERED", "Simulated side-road vehicle demand ON");
        } else {
            logEvent(nowMs, "DEMAND_INFO", "Simulated side-road vehicle demand already ON");
        }
        // If side road is currently green and gap timer was running, vehicle arrival resets gap timer
        if (state_ == TrafficState::SIDE_GREEN) {
            sideGapTimerActive_ = false;
        }
    } else {
        if (simVehiclePresent_) {
            simVehiclePresent_ = false;
            logEvent(nowMs, "DEMAND_CLEARED", "Simulated side-road vehicle demand OFF");
            if (state_ == TrafficState::SIDE_GREEN && !sideGapTimerActive_) {
                sideGapTimerActive_ = true;
                sideGapStartMs_     = nowMs;
            }
        } else {
            logEvent(nowMs, "DEMAND_INFO", "Simulated side-road vehicle demand already OFF");
        }
    }
    return true;
}

bool TrafficController::triggerSimPedestrianRequest(uint32_t nowMs) {
    if (mode_ != OperatingMode::SERIAL_SIMULATION) {
        logEvent(nowMs, "COMMAND_REJECTED", "Simulation command rejected in HARDWARE_READY mode (use SIM ON)");
        return false;
    }
    if (state_ == TrafficState::FAULT_SAFE) {
        logEvent(nowMs, "COMMAND_REJECTED", "Cannot inject demand in FAULT_SAFE state");
        return false;
    }

    if (!pedRequestLatched_) {
        pedRequestLatched_ = true;
        pedRequestFromSim_ = true;
        logEvent(nowMs, "PED_REQUEST_LATCHED", "Simulated pedestrian crossing request latched");
    } else {
        logEvent(nowMs, "PED_REQUEST_PENDING", "Simulated pedestrian request already latched");
    }
    return true;
}

bool TrafficController::clearSimulatedDemand(uint32_t nowMs) {
    if (mode_ != OperatingMode::SERIAL_SIMULATION) {
        logEvent(nowMs, "COMMAND_REJECTED", "CLEAR rejected in HARDWARE_READY mode (use SIM ON)");
        return false;
    }

    const bool hadVeh = simVehiclePresent_;
    const bool hadPed = pedRequestLatched_ && pedRequestFromSim_;

    simVehiclePresent_ = false;
    if (pedRequestFromSim_) {
        pedRequestLatched_ = false;
        pedRequestFromSim_ = false;
    }

    if (state_ == TrafficState::SIDE_GREEN && !sideGapTimerActive_) {
        sideGapTimerActive_ = true;
        sideGapStartMs_     = nowMs;
    }

    if (hadVeh || hadPed) {
        logEvent(nowMs, "DEMAND_CLEARED", "Cleared pending simulated demand(s)");
    } else {
        logEvent(nowMs, "DEMAND_CLEARED", "No simulated demands were pending");
    }
    return true;
}

void TrafficController::resetLogical(uint32_t nowMs, const char* reason) {
    // Clear demands and timers without restarting MCU or causing conflicting greens
    pedRequestLatched_  = false;
    pedRequestFromSim_  = false;
    simVehiclePresent_  = false;
    hwVehicleLatched_   = false;
    sideGapTimerActive_ = false;
    faultReason_        = "NONE";
    nextTarget_         = NextPhaseTarget::MAIN_ROAD;
    lastServedNonMain_  = TrafficState::STARTUP_SAFE;

    hwBtnLastRawPressed_ = false;
    hwBtnStablePressed_  = false;
    hwBtnAwaitRelease_   = false;
    hwBtnLastChangeMs_   = nowMs;
    hwIrLastRawDetected_ = false;
    hwIrStableDetected_  = false;
    hwIrStuckFault_      = false;
    hwIrLastChangeMs_    = nowMs;
    hwIrActiveStartMs_   = nowMs;

    logEvent(nowMs, "LOGICAL_RESET", reason);
    // Enter STARTUP_SAFE directly so all outputs are LOW -> ALL_RED -> MAIN_GREEN
    // This guarantees no momentary conflicting green from any previous phase.
    const TrafficState prevState = state_;
    state_        = TrafficState::STARTUP_SAFE;
    stateStartMs_ = nowMs;
    if (transitionCb_ && prevState != TrafficState::STARTUP_SAFE) {
        transitionCb_(nowMs, prevState, TrafficState::STARTUP_SAFE, reason);
    }
}

void TrafficController::triggerFault(uint32_t nowMs, const char* reason) {
    faultReason_ = reason ? reason : "INTERNAL_CONTROL_FAULT";
    logEvent(nowMs, "FAULT_DETECTED", faultReason_);
    if (state_ != TrafficState::FAULT_SAFE) {
        const TrafficState prev = state_;
        state_        = TrafficState::FAULT_SAFE;
        stateStartMs_ = nowMs;
        if (transitionCb_) {
            transitionCb_(nowMs, prev, TrafficState::FAULT_SAFE, faultReason_);
        }
    }
}

NextPhaseTarget TrafficController::selectNextTargetAfterMain() const {
    const bool pedPending  = pedRequestLatched_;
    const bool sidePending = hasSideVehicleDemand();

    if (pedPending && sidePending) {
        // Deterministic round-robin arbitration when both requests are pending:
        // If Pedestrian was served most recently, serve Side Road next; otherwise Pedestrian first.
        if (lastServedNonMain_ == TrafficState::PED_WALK) {
            return NextPhaseTarget::SIDE_ROAD;
        }
        return NextPhaseTarget::PEDESTRIAN;
    }
    if (pedPending) {
        return NextPhaseTarget::PEDESTRIAN;
    }
    if (sidePending) {
        return NextPhaseTarget::SIDE_ROAD;
    }
    return NextPhaseTarget::MAIN_ROAD;
}

void TrafficController::update(uint32_t nowMs) {
    // Verify output invariants before running state progression
    const SignalOutputs currentOut = getOutputs();
    const char* violation = nullptr;
    if (!verifySafetyInvariants(state_, currentOut, &violation)) {
        triggerFault(nowMs, violation ? violation : "SAFETY_INVARIANT_VIOLATION");
        return;
    }

    if (state_ == TrafficState::FAULT_SAFE) {
        return;
    }

    // Rollover-safe unsigned subtraction
    const uint32_t elapsed = nowMs - stateStartMs_;

    switch (state_) {
        case TrafficState::STARTUP_SAFE: {
            if (elapsed >= timings_.startupSafeMs) {
                nextTarget_ = NextPhaseTarget::MAIN_ROAD;
                transitionTo(nowMs, TrafficState::ALL_RED, "STARTUP_CLEARANCE_COMPLETE");
            }
            break;
        }

        case TrafficState::ALL_RED: {
            if (elapsed >= timings_.allRedMs) {
                if (nextTarget_ == NextPhaseTarget::SIDE_ROAD) {
                    sideGapTimerActive_ = !hasSideVehicleDemand();
                    sideGapStartMs_     = nowMs;
                    transitionTo(nowMs, TrafficState::SIDE_GREEN, "ALL_RED_CLEARANCE_COMPLETE");
                } else if (nextTarget_ == NextPhaseTarget::PEDESTRIAN) {
                    transitionTo(nowMs, TrafficState::PED_WALK, "ALL_RED_CLEARANCE_COMPLETE");
                } else {
                    transitionTo(nowMs, TrafficState::MAIN_GREEN, "ALL_RED_CLEARANCE_COMPLETE");
                }
            }
            break;
        }

        case TrafficState::MAIN_GREEN: {
            // Respect minimum main-road green duration before responding to any demand
            if (elapsed >= timings_.mainMinGreenMs) {
                const NextPhaseTarget target = selectNextTargetAfterMain();
                if (target == NextPhaseTarget::PEDESTRIAN) {
                    nextTarget_ = NextPhaseTarget::PEDESTRIAN;
                    transitionTo(nowMs, TrafficState::MAIN_YELLOW, "PEDESTRIAN_DEMAND");
                } else if (target == NextPhaseTarget::SIDE_ROAD) {
                    nextTarget_ = NextPhaseTarget::SIDE_ROAD;
                    transitionTo(nowMs, TrafficState::MAIN_YELLOW, "SIDE_ROAD_DEMAND");
                }
            }
            break;
        }

        case TrafficState::MAIN_YELLOW: {
            if (elapsed >= timings_.mainYellowMs) {
                // Re-evaluate target in case higher-priority or additional request arrived,
                // or if simulation demand was cleared before entering ALL_RED.
                const NextPhaseTarget evaluated = selectNextTargetAfterMain();
                nextTarget_ = evaluated;
                transitionTo(nowMs, TrafficState::ALL_RED, "MAIN_YELLOW_COMPLETE");
            }
            break;
        }

        case TrafficState::SIDE_GREEN: {
            // Check whether vehicle presence is currently active
            const bool activePresence = (mode_ == OperatingMode::SERIAL_SIMULATION)
                                            ? simVehiclePresent_
                                            : (physicalIrValidated_ && !hwIrStuckFault_ && hwIrStableDetected_);

            if (activePresence) {
                sideGapTimerActive_ = false;
            } else if (!sideGapTimerActive_) {
                sideGapTimerActive_ = true;
                sideGapStartMs_     = nowMs;
            }

            const uint32_t gapElapsed = sideGapTimerActive_ ? (nowMs - sideGapStartMs_) : 0;

            // Condition 1: Maximum green reached
            if (elapsed >= timings_.sideMaxGreenMs) {
                hwVehicleLatched_  = false;
                lastServedNonMain_ = TrafficState::SIDE_GREEN;
                nextTarget_        = pedRequestLatched_ ? NextPhaseTarget::PEDESTRIAN : NextPhaseTarget::MAIN_ROAD;
                logEvent(nowMs, "SERVICE_COMPLETED", "Side-road service ended (MAX_GREEN reached)");
                transitionTo(nowMs, TrafficState::SIDE_YELLOW, "SIDE_MAX_GREEN_ELAPSED");
            }
            // Condition 2: Minimum green elapsed AND demand cleared for at least clear-gap interval
            else if (elapsed >= timings_.sideMinGreenMs &&
                     sideGapTimerActive_ &&
                     gapElapsed >= timings_.sideClearGapMs) {
                hwVehicleLatched_  = false;
                lastServedNonMain_ = TrafficState::SIDE_GREEN;
                nextTarget_        = pedRequestLatched_ ? NextPhaseTarget::PEDESTRIAN : NextPhaseTarget::MAIN_ROAD;
                logEvent(nowMs, "SERVICE_COMPLETED", "Side-road service ended (demand cleared + gap elapsed)");
                transitionTo(nowMs, TrafficState::SIDE_YELLOW, "SIDE_DEMAND_GAP_CLEARED");
            }
            break;
        }

        case TrafficState::SIDE_YELLOW: {
            if (elapsed >= timings_.sideYellowMs) {
                // Transition through SIDE_YELLOW and ALL_RED before restoring MAIN_GREEN
                nextTarget_ = NextPhaseTarget::MAIN_ROAD;
                transitionTo(nowMs, TrafficState::ALL_RED, "SIDE_YELLOW_COMPLETE");
            }
            break;
        }

        case TrafficState::PED_WALK: {
            if (elapsed >= timings_.pedWalkMs) {
                pedRequestLatched_ = false;
                pedRequestFromSim_ = false;
                lastServedNonMain_ = TrafficState::PED_WALK;
                nextTarget_        = NextPhaseTarget::MAIN_ROAD;
                logEvent(nowMs, "SERVICE_COMPLETED", "Pedestrian WALK phase completed; request unlatched");
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
            return (to == TrafficState::MAIN_GREEN ||
                    to == TrafficState::SIDE_GREEN ||
                    to == TrafficState::PED_WALK);
        case TrafficState::MAIN_GREEN:
            return (to == TrafficState::MAIN_YELLOW);
        case TrafficState::MAIN_YELLOW:
            return (to == TrafficState::ALL_RED);
        case TrafficState::SIDE_GREEN:
            return (to == TrafficState::SIDE_YELLOW);
        case TrafficState::SIDE_YELLOW:
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
        triggerFault(nowMs, "INVALID_PHASE_TRANSITION");
        return;
    }

    const SignalOutputs candidateOut = deriveOutputsForState(newState);
    const char* violation = nullptr;
    if (!verifySafetyInvariants(newState, candidateOut, &violation)) {
        triggerFault(nowMs, violation ? violation : "CANDIDATE_OUTPUT_INVARIANT_VIOLATION");
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
    return deriveOutputsForState(state_);
}

SignalOutputs TrafficController::deriveOutputsForState(TrafficState state) {
    // Default all outputs LOW
    SignalOutputs out = {false, false, false, false, false, false, false, false};

    switch (state) {
        case TrafficState::STARTUP_SAFE:
            // Do not energize external outputs during startup safe state
            break;

        case TrafficState::ALL_RED:
            out.mainRed     = true;
            out.sideRed     = true;
            out.pedDontWalk = true;
            break;

        case TrafficState::MAIN_GREEN:
            out.mainGreen   = true;
            out.sideRed     = true;
            out.pedDontWalk = true;
            break;

        case TrafficState::MAIN_YELLOW:
            out.mainYellow  = true;
            out.sideRed     = true;
            out.pedDontWalk = true;
            break;

        case TrafficState::SIDE_GREEN:
            out.mainRed     = true;
            out.sideGreen   = true;
            out.pedDontWalk = true;
            break;

        case TrafficState::SIDE_YELLOW:
            out.mainRed     = true;
            out.sideYellow  = true;
            out.pedDontWalk = true;
            break;

        case TrafficState::PED_WALK:
            out.mainRed     = true;
            out.sideRed     = true;
            out.pedWalk     = true;
            break;

        case TrafficState::FAULT_SAFE:
        default:
            // In FAULT_SAFE, command all configured outputs LOW
            break;
    }

    return out;
}

bool TrafficController::verifySafetyInvariants(TrafficState state, const SignalOutputs& out, const char** violationReason) {
    // Invariant 1: Main-road GREEN and side-road GREEN must never be active simultaneously.
    if (out.mainGreen && out.sideGreen) {
        if (violationReason) *violationReason = "CONFLICTING_VEHICLE_GREENS";
        return false;
    }

    // Invariant 2: Pedestrian WALK must never be active simultaneously with either vehicle GREEN.
    if (out.pedWalk && (out.mainGreen || out.sideGreen)) {
        if (violationReason) *violationReason = "WALK_CONCURRENT_WITH_VEHICLE_GREEN";
        return false;
    }

    // Invariant 3: All vehicle approaches must be logically RED during pedestrian WALK.
    if (state == TrafficState::PED_WALK || out.pedWalk) {
        if (!out.mainRed || !out.sideRed || out.mainGreen || out.mainYellow || out.sideGreen || out.sideYellow) {
            if (violationReason) *violationReason = "VEHICLES_NOT_RED_DURING_PED_WALK";
            return false;
        }
        if (out.pedDontWalk) {
            if (violationReason) *violationReason = "PED_WALK_AND_DONT_WALK_CONCURRENT";
            return false;
        }
    }

    // Invariant 4: Yellow must never be displayed simultaneously with the same approach's green (or red).
    if ((out.mainYellow && out.mainGreen) || (out.mainYellow && out.mainRed)) {
        if (violationReason) *violationReason = "MAIN_APPROACH_CONFLICTING_ASPECTS";
        return false;
    }
    if ((out.sideYellow && out.sideGreen) || (out.sideYellow && out.sideRed)) {
        if (violationReason) *violationReason = "SIDE_APPROACH_CONFLICTING_ASPECTS";
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

    // Invariant 10: In FAULT_SAFE (and STARTUP_SAFE), all configured outputs must be LOW.
    if (state == TrafficState::FAULT_SAFE || state == TrafficState::STARTUP_SAFE) {
        if (out.mainGreen || out.mainYellow || out.mainRed ||
            out.sideGreen || out.sideYellow || out.sideRed ||
            out.pedWalk   || out.pedDontWalk) {
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

const char* TrafficController::targetToString(NextPhaseTarget t) {
    switch (t) {
        case NextPhaseTarget::MAIN_ROAD:  return "MAIN_ROAD";
        case NextPhaseTarget::SIDE_ROAD:  return "SIDE_ROAD";
        case NextPhaseTarget::PEDESTRIAN: return "PEDESTRIAN";
        default:                          return "UNKNOWN";
    }
}
