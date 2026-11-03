#include <Arduino.h>
#include <string.h>
#include <ctype.h>
#include "TrafficConfig.h"
#include "TrafficController.h"
#include "SerialCommandHandler.h"

#ifndef PIO_UNIT_TESTING
namespace {

TrafficController    controller;
SerialCommandHandler serialHandler;
TrafficState         lastAppliedState = TrafficState::STARTUP_SAFE;
bool                 firstOutputPass  = true;

constexpr size_t CMD_BUFFER_SIZE = 96;
char   cmdBuffer[CMD_BUFFER_SIZE];
size_t cmdIndex = 0;

// ============================================================================
// CENTRALIZED GPIO INITIALIZATION & HARDWARE ABSTRACTION
// ============================================================================
void initializeGpioSafe() {
    // 1. Initialize all 13 signal output channels via SignalOutputDriver
    //    Sets each pin to its electrical OFF level BEFORE enabling OUTPUT mode.
    controller.getOutputDriver().initializeHardwareSafe();

    // 2. Configure 2 pedestrian pushbutton inputs with internal pull-ups
    pinMode(TrafficConfig::PIN_PED1_BUTTON, INPUT_PULLUP);
    pinMode(TrafficConfig::PIN_PED2_BUTTON, INPUT_PULLUP);

    // 3. Configure 6 IR vehicle sensor inputs:
    //    - GPIOs 33 & 15 support internal pull-ups (INPUT_PULLUP).
    //    - GPIOs 34, 35, 36, 39 are input-only and require external 10k pull-ups to 3.3V (INPUT).
    for (uint8_t i = 0; i < TrafficConfig::IR_SENSOR_COUNT; ++i) {
        if (TrafficConfig::IR_SENSOR_HAS_INTERNAL_PULLUP[i]) {
            pinMode(TrafficConfig::IR_SENSOR_PINS[i], INPUT_PULLUP);
        } else {
            pinMode(TrafficConfig::IR_SENSOR_PINS[i], INPUT);
        }
    }
}

void applySignalOutputsFromFsm() {
    const TrafficState currentState = controller.getState();
    const SignalOutputs out         = controller.getOutputs();
    const char* violation           = nullptr;

    if (!TrafficController::verifySafetyInvariants(currentState, out, &violation)) {
        controller.getOutputDriver().forceAllSafeOff();
        controller.triggerFault(millis(),
                                violation ? violation : "OUTPUT_SAFETY_CHECK_FAILED",
                                FaultCode::ILLEGAL_SIGNAL_COMBINATION);
        return;
    }

    if (!firstOutputPass && currentState == lastAppliedState && currentState != TrafficState::PED_WALK) {
        return;
    }

    controller.getOutputDriver().applyLogicalOutputs(out);
    lastAppliedState = currentState;
    firstOutputPass  = false;
}

// ============================================================================
// DIAGNOSTICS, TELEMETRY & SERIAL PRINTERS
// ============================================================================
void onStateTransition(uint32_t nowMs, TrafficState from, TrafficState to, const char* reason) {
    Serial.printf("[%lu ms] %s -> %s | Reason: %s\r\n",
                  static_cast<unsigned long>(nowMs),
                  TrafficController::stateToString(from),
                  TrafficController::stateToString(to),
                  reason ? reason : "NONE");

    if (to == TrafficState::FAULT_SAFE) {
        Serial.println(F("[FAULT_SAFE] All logical signal outputs forced OFF; hardware driver set to safe electrical state."));
        Serial.println(F("[WARNING] Physical fail-safe lamp state requires verified hardware driver circuitry."));
    }
}

void onControllerEvent(uint32_t nowMs, const char* eventType, const char* detail) {
    Serial.printf("[%lu ms] EVENT: %s | %s\r\n",
                  static_cast<unsigned long>(nowMs),
                  eventType ? eventType : "INFO",
                  detail ? detail : "");
}

void printHelp() {
    Serial.println(F("------------------------------------------------------------------------"));
    Serial.println(F("AVAILABLE SERIAL COMMANDS:"));
    Serial.println(F("  HELP          - List all supported serial commands"));
    Serial.println(F("  STATUS        - Print controller state, mode, timers, and outputs"));
    Serial.println(F("  MODE AUTO     - Automatic mode (actuated + fair demand rotation)"));
    Serial.println(F("  MODE FIXED    - Fixed-time cyclic phase operation"));
    Serial.println(F("  MODE ACTUATED - Vehicle-actuated operation across 6 IR sensors"));
    Serial.println(F("  MODE ADAPTIVE - Adaptive demand & waiting-time priority scheduling"));
    Serial.println(F("  MODE MANUAL   - Manual operator mode (use NEXT to step safely)"));
    Serial.println(F("  NEXT          - Advance to next phase in MANUAL mode via yellow/all-red"));
    Serial.println(F("  PED 1 / PED 2 - Queue pedestrian crossing 1 or 2 request"));
    Serial.println(F("  CAR ON / OFF  - Set/clear simulated side-road vehicle demand"));
    Serial.println(F("  CAR MAIN ON/OFF | CAR TURN ON/OFF - Set/clear Main or Turn demand"));
    Serial.println(F("  SIM ON / OFF  - Enable/disable serial simulation input mode"));
    Serial.println(F("  FAULT         - Trigger fault-safe state immediately"));
    Serial.println(F("  CLEAR         - Recover from FAULT_SAFE or clear simulated demand"));
    Serial.println(F("  RESET         - Controlled logical reset to STARTUP_SAFE"));
    Serial.println(F("  SENSORS       - Print status of all 6 IR sensors & 2 Ped buttons"));
    Serial.println(F("  DEMAND        - Print approach demand, waiting time, and adaptive scores"));
    Serial.println(F("  PHASES        - Print movement phases & Keep-Right conflict matrix"));
    Serial.println(F("  TIMING        - Print active phase timing intervals"));
    Serial.println(F("  CONFIG        - Print hardware driver & persistent config summary"));
    Serial.println(F("  SAVE / LOAD   - Save or load configuration in ESP32 NVS"));
    Serial.println(F("  DEFAULTS      - Restore and save default configuration"));
    Serial.println(F("  REBOOT        - Safe logical reset followed by ESP32 restart"));
    Serial.println(F("------------------------------------------------------------------------"));
}

void printStatus(uint32_t nowMs) {
    const TrafficState st   = controller.getState();
    const SignalOutputs out = controller.getOutputs();
    Serial.println(F("================ CONTROLLER STATUS ================"));
    Serial.printf("  Timestamp         : %lu ms\r\n", static_cast<unsigned long>(nowMs));
    Serial.printf("  Operating Mode    : %s | Control Mode: %s\r\n",
                  TrafficController::modeToString(controller.getMode()),
                  TrafficController::controlModeToString(controller.getControlMode()));
    Serial.printf("  Current FSM State : %s (Elapsed: %lu ms)\r\n",
                  TrafficController::stateToString(st),
                  static_cast<unsigned long>(controller.getStateElapsed(nowMs)));
    Serial.printf("  Next Target Phase : %s\r\n", TrafficController::targetToString(controller.getNextTarget()));
    Serial.printf("  Protected Turn    : %s (Keep-Right rule)\r\n",
                  controller.isProtectedTurnCommissioned() ? "COMMISSIONED" : "UNCOMMISSIONED_DISABLED");
    Serial.printf("  Vehicle Demands   : MAIN=%s | SIDE=%s | TURN=%s\r\n",
                  controller.hasMainVehicleDemand() ? "YES" : "NO",
                  controller.hasSideVehicleDemand() ? "YES" : "NO",
                  controller.hasTurnVehicleDemand() ? "YES" : "NO");
    Serial.printf("  Pedestrian Demand : PED1=%s | PED2=%s | Clearance=%s\r\n",
                  controller.hasPedestrianChannelDemand(0) ? "LATCHED" : "NONE",
                  controller.hasPedestrianChannelDemand(1) ? "LATCHED" : "NONE",
                  controller.isPedestrianClearanceActive() ? "ACTIVE" : "NO");
    Serial.printf("  Logical Outputs   : MAIN[G=%d Y=%d R=%d] SIDE[G=%d Y=%d R=%d] TURN[G=%d Y=%d R=%d] PED1[W=%d DW=%d] PED2[W=%d DW=%d]\r\n",
                  out.mainGreen, out.mainYellow, out.mainRed,
                  out.sideGreen, out.sideYellow, out.sideRed,
                  out.turnGreen, out.turnYellow, out.turnRed,
                  out.pedWalk, out.pedDontWalk,
                  out.ped2Walk, out.ped2DontWalk);
    Serial.printf("  Active Fault Mask : 0x%04X | Last Fault: %s\r\n",
                  controller.getFaultManager().getActiveFaultMask(),
                  controller.getFaultReason());
    Serial.println(F("==================================================="));
}

void printSensors() {
    Serial.println(F("================ SENSOR & BUTTON STATUS ================"));
    const VehicleDetector& det = controller.getVehicleDetector();
    for (uint8_t i = 0; i < TrafficConfig::IR_SENSOR_COUNT; ++i) {
        const IrChannelStatus& ch = det.getChannel(i);
        Serial.printf("  IR%u (GPIO %2u) -> Mov=%u | Validated=%s | Stable=%d | Latched=%d | Stuck=%d | Chatter=%d\r\n",
                      static_cast<unsigned>(i + 1),
                      TrafficConfig::IR_SENSOR_PINS[i],
                      ch.assignedMovement,
                      ch.validated ? "YES" : "NO",
                      ch.stableDetected,
                      ch.latchedDemand,
                      ch.stuckFault,
                      ch.chatterFault);
    }
    const PedestrianController& ped = controller.getPedestrianController();
    for (uint8_t b = 0; b < TrafficConfig::PED_BUTTON_COUNT; ++b) {
        const PedChannelState& pch = ped.getChannel(b);
        const uint8_t pin = (b == 0) ? TrafficConfig::PIN_PED1_BUTTON : TrafficConfig::PIN_PED2_BUTTON;
        Serial.printf("  PED%u_BTN (GPIO %2u) | Pressed=%d | Latched=%d | Stuck=%d\r\n",
                      static_cast<unsigned>(b + 1),
                      pin,
                      pch.stablePressed,
                      pch.requestLatched,
                      pch.stuckFault);
    }
    Serial.println(F("========================================================"));
}

void printDemand() {
    Serial.println(F("================ APPROACH DEMAND & SCORES ================"));
    const DemandManager& dm = controller.getDemandManager();
    const char* names[3] = { "MAIN_ROAD", "SIDE_ROAD", "PROTECTED_TURN" };
    for (uint8_t i = 0; i < TrafficConfig::VEHICLE_GROUP_COUNT; ++i) {
        const TrafficConfig::MovementId m = static_cast<TrafficConfig::MovementId>(i);
        const ApproachDemandState& st = dm.getApproachState(m);
        Serial.printf("  %-14s : Demand=%s | Sim=%s | Sensors=%u | Wait=%lu ms | Skipped=%u | Score=%lu\r\n",
                      names[i],
                      st.hasDemand ? "YES" : "NO",
                      st.simDemandActive ? "ON" : "OFF",
                      st.activeSensorCount,
                      static_cast<unsigned long>(st.waitingTimeMs),
                      st.skippedCycles,
                      static_cast<unsigned long>(st.lastScore));
    }
    Serial.printf("  PED_CROSSING_1 : %s\r\n", controller.hasPedestrianChannelDemand(0) ? "LATCHED" : "NONE");
    Serial.printf("  PED_CROSSING_2 : %s\r\n", controller.hasPedestrianChannelDemand(1) ? "LATCHED" : "NONE");
    Serial.println(F("=========================================================="));
}

void printPhases() {
    Serial.println(F("================ PHASES & KEEP-RIGHT CONFLICT MATRIX ================"));
    const TrafficConfig::ConflictMatrixConfig& cm = controller.getConflictMatrixConfig();
    Serial.printf("  Traffic Rule              : %s\r\n", cm.rightHandTrafficRule ? "RIGHT-HAND (KEEP-RIGHT)" : "INVALID");
    Serial.printf("  Protected Turn Status     : %s\r\n",
                  cm.protectedTurnCommissioned ? "COMMISSIONED" : "DISABLED (Pending physical geometry confirmation)");
    const char* labels[5] = { "MAIN", "SIDE", "TURN", "PED1", "PED2" };
    Serial.println(F("  Conflict Matrix (1 = Conflicting / Forbidden Concurrent):"));
    Serial.println(F("         MAIN  SIDE  TURN  PED1  PED2"));
    for (uint8_t r = 0; r < 5; ++r) {
        Serial.printf("  %-4s :   %d     %d     %d     %d     %d\r\n",
                      labels[r],
                      cm.conflicts[r][0], cm.conflicts[r][1], cm.conflicts[r][2],
                      cm.conflicts[r][3], cm.conflicts[r][4]);
    }
    Serial.println(F("====================================================================="));
}

void printTiming() {
    const TrafficConfig::TimingConfig& t = controller.getTimingConfig();
    Serial.println(F("================ ACTIVE TIMING CONFIGURATION ================"));
    Serial.printf("  Startup Safe   : %lu ms | All-Red Clearance : %lu ms\r\n",
                  static_cast<unsigned long>(t.startupSafeMs), static_cast<unsigned long>(t.allRedMs));
    Serial.printf("  Main Road      : MinG=%lu ms | MaxG=%lu ms | Gap=%lu ms | Yellow=%lu ms\r\n",
                  static_cast<unsigned long>(t.mainMinGreenMs), static_cast<unsigned long>(t.mainMaxGreenMs),
                  static_cast<unsigned long>(t.mainClearGapMs), static_cast<unsigned long>(t.mainYellowMs));
    Serial.printf("  Side Road      : MinG=%lu ms | MaxG=%lu ms | Gap=%lu ms | Yellow=%lu ms\r\n",
                  static_cast<unsigned long>(t.sideMinGreenMs), static_cast<unsigned long>(t.sideMaxGreenMs),
                  static_cast<unsigned long>(t.sideClearGapMs), static_cast<unsigned long>(t.sideYellowMs));
    Serial.printf("  Protected Turn : MinG=%lu ms | MaxG=%lu ms | Gap=%lu ms | Yellow=%lu ms\r\n",
                  static_cast<unsigned long>(t.turnMinGreenMs), static_cast<unsigned long>(t.turnMaxGreenMs),
                  static_cast<unsigned long>(t.turnClearGapMs), static_cast<unsigned long>(t.turnYellowMs));
    Serial.printf("  Pedestrian     : Walk=%lu ms | Clearance=%lu ms\r\n",
                  static_cast<unsigned long>(t.pedWalkMs), static_cast<unsigned long>(t.pedClearanceMs));
    Serial.println(F("============================================================="));
}

void printConfig() {
    const SignalOutputDriver& drv = controller.getOutputDriver();
    const TrafficConfig::SensorConfig& sc = controller.getSensorConfig();
    const TrafficConfig::DemandConfig& dc = controller.getDemandConfig();
    Serial.println(F("================ HARDWARE & CONTROLLER CONFIG ================"));
    Serial.printf("  Control Mode          : %s\r\n", TrafficController::controlModeToString(controller.getControlMode()));
    Serial.printf("  Vehicle Driver Type   : %s (Verified=%s)\r\n",
                  SignalOutputDriver::driverTypeToString(drv.getVehicleDriverType()),
                  drv.isVehicleDriverVerified() ? "YES" : "NO [5V Common-GND modules require high-side source]");
    Serial.printf("  Pedestrian Driver     : %s (Verified=%s)\r\n",
                  SignalOutputDriver::driverTypeToString(drv.getPedestrianDriverType()),
                  drv.isPedestrianDriverVerified() ? "YES" : "NO [ULN2003A Common-Anode]");
    Serial.printf("  Side Yellow Pin       : GPIO %u (Safe proposed alternative: GPIO %u)\r\n",
                  drv.getSideYellowPin(), TrafficConfig::PIN_SIDE_YELLOW_SAFE_PROPOSED);
    Serial.printf("  Input Filtering       : IR Stability=%lu ms | Ped Debounce=%lu ms | Stuck Timeout=%lu ms\r\n",
                  static_cast<unsigned long>(sc.irStabilityMs),
                  static_cast<unsigned long>(sc.pedDebounceMs),
                  static_cast<unsigned long>(sc.maxInputStuckMs));
    Serial.printf("  Adaptive Weights      : SensorWeight=%u | WaitWeight=%lu ms | StarvationLimit=%u cycles\r\n",
                  dc.activeSensorWeight,
                  static_cast<unsigned long>(dc.waitTimeWeightMs),
                  dc.starvationSkipLimit);
    Serial.println(F("=============================================================="));
}

void onSerialPrinterMarker(const char* marker) {
    if (!marker) return;
    const uint32_t nowMs = millis();
    if (strcmp(marker, "HELP_PRINTED") == 0) {
        printHelp();
    } else if (strcmp(marker, "STATUS_PRINTED") == 0) {
        printStatus(nowMs);
    } else if (strcmp(marker, "SENSORS_PRINTED") == 0) {
        printSensors();
    } else if (strcmp(marker, "DEMAND_PRINTED") == 0) {
        printDemand();
    } else if (strcmp(marker, "PHASES_PRINTED") == 0) {
        printPhases();
    } else if (strcmp(marker, "TIMING_PRINTED") == 0) {
        printTiming();
    } else if (strcmp(marker, "CONFIG_PRINTED") == 0) {
        printConfig();
    }
}

void printStartupBanner(uint32_t nowMs) {
    Serial.println();
    Serial.println(F("========================================================================"));
    Serial.printf("PROJECT  : %s\r\n", TrafficConfig::PROJECT_NAME);
    Serial.printf("VERSION  : %s\r\n", TrafficConfig::FIRMWARE_VER);
    Serial.printf("MODE     : %s | Control: %s\r\n",
                  TrafficController::modeToString(controller.getMode()),
                  TrafficController::controlModeToString(controller.getControlMode()));
    Serial.printf("STATE    : %s (Initial Logical State at %lu ms)\r\n",
                  TrafficController::stateToString(controller.getState()),
                  static_cast<unsigned long>(nowMs));
    Serial.println(F("------------------------------------------------------------------------"));
    Serial.println(F("IMPORTANT HARDWARE & ELECTRICAL SAFETY WARNINGS:"));
    Serial.println(F("  [!] Vehicle RYG modules are 5V COMMON-GROUND (+5V sourcing required)."));
    Serial.println(F("      Do NOT wire vehicle modules directly to ULN2003A low-side outputs."));
    Serial.println(F("  [!] Protected Turn phase is UNCOMMISSIONED by default until physical"));
    Serial.println(F("      Keep-Right junction turning paths are confirmed."));
    Serial.println(F("  [!] Input-only GPIOs 34, 35, 36, 39 require external 10k pull-ups to 3.3V."));
    printHelp();
}

void pollSerialInput(uint32_t nowMs) {
    while (Serial.available() > 0) {
        const char c = static_cast<char>(Serial.read());
        if (c == '\r' || c == '\n') {
            if (cmdIndex > 0) {
                cmdBuffer[cmdIndex] = '\0';
                const SerialCommandResult res = serialHandler.executeCommand(nowMs, cmdBuffer, controller);
                if (res == SerialCommandResult::UNKNOWN_INVALID_CMD) {
                    Serial.printf("[%lu ms] EVENT: COMMAND_REJECTED | Unknown command: '%s' (Type HELP)\r\n",
                                  static_cast<unsigned long>(nowMs), cmdBuffer);
                }
                applySignalOutputsFromFsm();
                if (serialHandler.isRebootRequested()) {
                    serialHandler.clearRebootRequest();
                    Serial.println(F("[REBOOT] Restarting ESP32 controller..."));
                    Serial.flush();
                    ESP.restart();
                }
                cmdIndex = 0;
            }
        } else if (cmdIndex < (CMD_BUFFER_SIZE - 1)) {
            cmdBuffer[cmdIndex++] = c;
        } else {
            cmdIndex = 0;
            Serial.printf("[%lu ms] EVENT: COMMAND_REJECTED | Serial command too long\r\n",
                          static_cast<unsigned long>(nowMs));
        }
    }
}

HardwareInputs readHardwareAdapter() {
    HardwareInputs in;
    in.rawPedButtonPinHigh  = (digitalRead(TrafficConfig::PIN_PED1_BUTTON) == HIGH);
    in.rawPed2ButtonPinHigh = (digitalRead(TrafficConfig::PIN_PED2_BUTTON) == HIGH);
    in.rawIrSensorPinHigh   = (digitalRead(TrafficConfig::PIN_IR_SENSOR_1) == HIGH);
    in.inputsValid          = true;
    in.hasExtendedChannels  = true;
    for (uint8_t i = 0; i < TrafficConfig::IR_SENSOR_COUNT; ++i) {
        in.irPinHigh[i] = (digitalRead(TrafficConfig::IR_SENSOR_PINS[i]) == HIGH);
    }
    return in;
}

} // namespace

void setup() {
    initializeGpioSafe();
    Serial.begin(TrafficConfig::SERIAL_BAUD);

    const uint32_t nowMs = millis();
    controller.setTransitionCallback(onStateTransition);
    controller.setEventLogCallback(onControllerEvent);
    serialHandler.setOutputCallback(onSerialPrinterMarker);
    controller.begin(nowMs);

    applySignalOutputsFromFsm();
    printStartupBanner(nowMs);
}

void loop() {
    const uint32_t nowMs = millis();

    pollSerialInput(nowMs);

    if (controller.getMode() == OperatingMode::HARDWARE_READY) {
        const HardwareInputs hwIn = readHardwareAdapter();
        controller.processHardwareAdapter(nowMs, hwIn);
    }

    controller.update(nowMs);
    applySignalOutputsFromFsm();
}
#endif // PIO_UNIT_TESTING
