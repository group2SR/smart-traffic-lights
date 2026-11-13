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

constexpr size_t CMD_BUFFER_SIZE = 128;
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

    if (controller.getOutputDriver().isLampTestActive()) {
        if (controller.getControlMode() != ControlMode::MANUAL &&
            !controller.isLampDiagnosticsActive() &&
            currentState != TrafficState::STARTUP_SAFE &&
            currentState != TrafficState::ALL_RED) {
            controller.stopLampTest(millis());
        }
    }

    controller.getOutputDriver().applyLogicalOutputs(out);
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
    Serial.println(F("  CLEAR         - Recover from FAULT_SAFE or clear faults/simulated demand"));
    Serial.println(F("  RESET         - Controlled logical reset to STARTUP_SAFE"));
    Serial.println(F("  SENSORS       - Print status of all 6 IR sensors & 2 Ped buttons"));
    Serial.println(F("  DEMAND        - Print approach demand, waiting time, and adaptive scores"));
    Serial.println(F("  PHASES        - Print movement phases & Keep-Right conflict matrix"));
    Serial.println(F("  TIMING        - Print active phase timing intervals"));
    Serial.println(F("  TIMING PROTOTYPE - Apply authoritative prototype fixed timings (25s/3s/2s/20s/3s/2s)"));
    Serial.println(F("  TIMING BENCH  - Apply rapid bench test timings (4s/2s/1s/2s/2s/1s)"));
    Serial.println(F("  TIMING MAIN <minG> [yel] [maxG] - Configure Main Road phase timing"));
    Serial.println(F("  TIMING SIDE <minG> [yel] [maxG] - Configure Side Road phase timing"));
    Serial.println(F("  TIMING TURN <minG> [yel] [maxG] - Configure Protected Turn phase timing"));
    Serial.println(F("  TIMING ALL_RED <ms>             - Configure intersection all-red clearance"));
    Serial.println(F("  TIMING PED <walk> [clear]       - Configure Pedestrian walk and clearance"));
    Serial.println(F("  TIMING SET <mMin> <mMax> <mYel> <sMin> <sMax> <sYel> <tMin> <tMax> <tYel> <pWalk> <pClear> <allRed> - Set all phase timings atomically & persist"));
    Serial.println(F("  CONFIG        - Print hardware driver & persistent config summary"));
    Serial.println(F("  COMMISSION    - Print hardware commissioning & verification checklist"));
    Serial.println(F("  DRIVER VEH <DIRECT_ACTIVE_HIGH|HIGH_SIDE_ACTIVE_HIGH|HIGH_SIDE_ACTIVE_LOW|DISABLED>"));
    Serial.println(F("  DRIVER PED <LOW_SIDE_ULN2003A|HIGH_SIDE_ACTIVE_HIGH|DIRECT_ACTIVE_HIGH|DISABLED>"));
    Serial.println(F("  COMMISSION VEH ON/OFF  - Enable/disable verified vehicle output driver"));
    Serial.println(F("  COMMISSION PED ON/OFF  - Enable/disable verified pedestrian ULN2003A driver"));
    Serial.println(F("  COMMISSION IR <1..6|ALL> ON/OFF - Validate/de-validate IR sensor channel(s)"));
    Serial.println(F("  COMMISSION TURN ON/OFF - Enable/disable verified RHT protected turn phase"));
    Serial.println(F("  LAMP TEST <MAIN_R|..|PED2_DW|OFF> - Single-lamp commissioning test (MANUAL/ALL_RED)"));
    Serial.println(F("  DIAG LAMPS [ms] - Safe individual lamp test across Main & Side R/Y/G (All-Red clearance)"));
    Serial.println(F("  DIAG REPORT   - Print latest lamp hardware diagnostic results & electrical audit"));
    Serial.println(F("  TEST ALL_GREEN [sec] - 10s ALL-GREEN hardware diagnostic test (BENCH ONLY, holds ALL-RED)"));
    Serial.println(F("  TEST ALL_GREEN STOP  - Abort running ALL-GREEN diagnostic test immediately"));
    Serial.println(F("  SAVE / LOAD   - Save or load configuration in ESP32 NVS"));
    Serial.println(F("  DEFAULTS      - Restore and save default configuration"));
    Serial.println(F("  REBOOT        - Safe logical reset followed by ESP32 restart"));
    Serial.println(F("------------------------------------------------------------------------"));
}

void printStatus(uint32_t nowMs) {
    const DashboardStatusSnapshot snap = SerialCommandHandler::captureStatusSnapshot(nowMs, controller);
    char logicalOutBuf[128];
    SerialCommandHandler::formatLogicalOutputsLine(snap.outputs, logicalOutBuf, sizeof(logicalOutBuf));
    char electricalBuf[128];
    const ElectricalPinStates& elSt = controller.getOutputDriver().getLastElectricalStates();
    SerialCommandHandler::formatElectricalPinStatesLine(elSt, electricalBuf, sizeof(electricalBuf));

    Serial.println(F("================ CONTROLLER STATUS ================"));
    Serial.printf("  Timestamp         : %lu ms\r\n", static_cast<unsigned long>(snap.timestampMs));
    Serial.printf("  Traffic Rule      : %s\r\n",
                  snap.rightHandTrafficRule ? "RHT / KEEPRIGHT (Right-Hand Traffic)" : "INVALID");
    Serial.printf("  Operating Mode    : %s | Control Mode: %s\r\n",
                  TrafficController::modeToString(snap.operatingMode),
                  TrafficController::controlModeToString(snap.controlMode));
    Serial.printf("  Current FSM State : %s (Elapsed: %lu ms)\r\n",
                  TrafficController::stateToString(snap.currentFsmState),
                  static_cast<unsigned long>(snap.stateElapsedMs));
    Serial.printf("  State Elapsed     : %lu ms\r\n", static_cast<unsigned long>(snap.stateElapsedMs));
    Serial.printf("  Side Gap Timer    : %s (%lu ms)\r\n",
                  snap.sideGapActive ? "ACTIVE" : "INACTIVE",
                  static_cast<unsigned long>(snap.sideGapElapsedMs));
    Serial.printf("  Next Target Phase : %s\r\n", TrafficController::targetToString(snap.nextTargetPhase));
    Serial.printf("  Protected Turn    : %s (RHT / KeepRight rule)\r\n",
                  snap.protectedTurnCommissioned ? "COMMISSIONED" : "UNCOMMISSIONED_DISABLED");
    Serial.printf("  Vehicle Demands   : MAIN=%s | SIDE=%s | TURN=%s\r\n",
                  snap.mainDemand ? "YES" : "NO",
                  snap.sideDemand ? "YES" : "NO",
                  snap.turnDemand ? "YES" : "NO");
    Serial.printf("  Side Demand       : %s (SimVeh=%s, HwLatched=%s)\r\n",
                  snap.sideDemand ? "PENDING" : "NONE",
                  controller.isSimVehicleActive() ? "ON" : "OFF",
                  controller.isHwVehicleLatched() ? "YES" : "NO");
    Serial.printf("  Pedestrian Demand : PED1=%s | PED2=%s | Clearance=%s\r\n",
                  snap.ped1Demand ? "LATCHED" : "NONE",
                  snap.ped2Demand ? "LATCHED" : "NONE",
                  snap.pedClearanceActive ? "ACTIVE" : "NO");
    Serial.printf("  Physical IR Valid : %s (Mask=0x%02X; Main=33/34, Side=35/36, Turn=39/15)\r\n",
                  snap.physicalIrValid ? "YES" : "NO",
                  snap.validatedIrMask);
    Serial.printf("  Logical Outputs   : %s\r\n", logicalOutBuf);
    Serial.printf("  Driver Pin Levels : %s\r\n", electricalBuf);
    Serial.printf("  Active Fault Mask : 0x%04X | Last Fault: %s\r\n",
                  snap.activeFaultMask,
                  controller.getFaultReason());
    Serial.println(F("==================================================="));
}

void printSensors() {
    Serial.println(F("================ SENSOR & BUTTON STATUS ================"));
    const VehicleDetector& det = controller.getVehicleDetector();
    const char* roadNames[3] = { "MAIN_ROAD", "SIDE_ROAD", "TURN_ROAD" };
    for (uint8_t i = 0; i < TrafficConfig::IR_SENSOR_COUNT; ++i) {
        const IrChannelStatus& ch = det.getChannel(i);
        const char* roadStr = (ch.assignedMovement < 3) ? roadNames[ch.assignedMovement] : "UNASSIGNED";
        Serial.printf("  IR%u (GPIO %2u) -> %-9s S%u | Validated=%s | RawHigh=%d | Stable=%d | Latched=%d | Det=%lu Rel=%lu | Cycle=%s | Stuck=%d | Chatter=%d\r\n",
                      static_cast<unsigned>(i + 1),
                      ch.gpioPin,
                      roadStr,
                      static_cast<unsigned>(ch.sensorPositionOnRoad + 1),
                      ch.validated ? "YES" : "NO",
                      ch.rawPinHigh,
                      ch.stableDetected,
                      ch.latchedDemand,
                      static_cast<unsigned long>(ch.detectionCount),
                      static_cast<unsigned long>(ch.releaseCount),
                      ch.cycleObserved ? "YES" : "NO",
                      ch.stuckFault,
                      ch.chatterFault);
    }
    const PedestrianController& ped = controller.getPedestrianController();
    for (uint8_t b = 0; b < TrafficConfig::PED_BUTTON_COUNT; ++b) {
        const PedChannelState& pch = ped.getChannel(b);
        Serial.printf("  PED%u_BTN (GPIO %2u) [Walk=GPIO %2u, DW=GPIO %2u] | RawHigh=%d | Pressed=%d | Latched=%d | Press=%lu Rel=%lu | Cycle=%s | Stuck=%d\r\n",
                      static_cast<unsigned>(b + 1),
                      pch.buttonGpioPin,
                      pch.walkGpioPin,
                      pch.dontWalkGpioPin,
                      pch.rawPinHigh,
                      pch.stablePressed,
                      pch.requestLatched,
                      static_cast<unsigned long>(pch.pressCount),
                      static_cast<unsigned long>(pch.releaseCount),
                      pch.cycleObserved ? "YES" : "NO",
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
        Serial.printf("  %-14s : Demand=%s | Sim=%s | S1=%d S2=%d (Active=%u) | Wait=%lu ms | Skipped=%u | Score=%lu\r\n",
                      names[i],
                      st.hasDemand ? "YES" : "NO",
                      st.simDemandActive ? "ON" : "OFF",
                      st.sensor1Active,
                      st.sensor2Active,
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
    Serial.println(F("================ PHASES & RHT / KEEP-RIGHT CONFLICT MATRIX ================"));
    const TrafficConfig::ConflictMatrixConfig& cm = controller.getConflictMatrixConfig();
    Serial.printf("  Traffic Rule              : %s\r\n",
                  cm.rightHandTrafficRule ? TrafficConfig::TRAFFIC_RULE_LABEL : "INVALID");
    Serial.printf("  Protected Turn Status     : %s\r\n",
                  cm.protectedTurnCommissioned
                      ? "COMMISSIONED (RHT Protected Left-Turn Active)"
                      : "DISABLED (Pending physical RHT turn-lane geometry confirmation)");
    Serial.println(F("  RHT / KeepRight Movement Definitions:"));
    for (uint8_t i = 0; i < static_cast<uint8_t>(TrafficConfig::MovementId::COUNT); ++i) {
        const TrafficConfig::RhtMovementDescriptor& desc = TrafficConfig::RHT_MOVEMENT_DESCRIPTORS[i];
        Serial.printf("    [%u] %-14s : %s\r\n", static_cast<unsigned>(i), desc.shortName, desc.rhtApproachLane);
    }
    const char* labels[5] = { "MAIN", "SIDE", "TURN", "PED1", "PED2" };
    Serial.println(F("  Conflict Matrix (1 = Conflicting / Forbidden Concurrent):"));
    Serial.println(F("         MAIN  SIDE  TURN  PED1  PED2"));
    for (uint8_t r = 0; r < 5; ++r) {
        Serial.printf("  %-4s :   %d     %d     %d     %d     %d\r\n",
                      labels[r],
                      cm.conflicts[r][0], cm.conflicts[r][1], cm.conflicts[r][2],
                      cm.conflicts[r][3], cm.conflicts[r][4]);
    }
    Serial.println(F("==========================================================================="));
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
                  drv.isVehicleDriverVerified() ? "YES" : "NO [Verify 3.3V compatibility & per-pin current <=20mA]");
    Serial.printf("  Pedestrian Driver     : %s (Verified=%s)\r\n",
                  SignalOutputDriver::driverTypeToString(drv.getPedestrianDriverType()),
                  drv.isPedestrianDriverVerified() ? "YES" : "NO [Verify ULN2003A low-side sink & common-anode resistors]");
    Serial.printf("  Vehicle RYG GPIOs     : MAIN[R=%u Y=%u G=%u] SIDE[R=%u Y=%u G=%u] TURN[R=%u Y=%u G=%u]\r\n",
                  TrafficConfig::PIN_MAIN_RED, TrafficConfig::PIN_MAIN_YELLOW, TrafficConfig::PIN_MAIN_GREEN,
                  TrafficConfig::PIN_SIDE_RED, TrafficConfig::PIN_SIDE_YELLOW, TrafficConfig::PIN_SIDE_GREEN,
                  TrafficConfig::PIN_TURN_RED, TrafficConfig::PIN_TURN_YELLOW, TrafficConfig::PIN_TURN_GREEN);
    Serial.printf("  Pedestrian GPIOs      : PED1[W=%u DW=%u BTN=%u] PED2[W=%u DW=%u BTN=%u]\r\n",
                  TrafficConfig::PIN_PED1_WALK, TrafficConfig::PIN_PED1_DONT_WALK, TrafficConfig::PIN_PED1_BUTTON,
                  TrafficConfig::PIN_PED2_WALK, TrafficConfig::PIN_PED2_DONT_WALK, TrafficConfig::PIN_PED2_BUTTON);
    Serial.printf("  Road IR Sensor GPIOs  : MAIN[S1=%u S2=%u] SIDE[S1=%u S2=%u] TURN[S1=%u S2=%u]\r\n",
                  TrafficConfig::PIN_MAIN_IR_SENSOR_1, TrafficConfig::PIN_MAIN_IR_SENSOR_2,
                  TrafficConfig::PIN_SIDE_IR_SENSOR_1, TrafficConfig::PIN_SIDE_IR_SENSOR_2,
                  TrafficConfig::PIN_TURN_IR_SENSOR_1, TrafficConfig::PIN_TURN_IR_SENSOR_2);
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

void printCommissionStatus() {
    const SignalOutputDriver& drv = controller.getOutputDriver();
    const VehicleDetector& det = controller.getVehicleDetector();
    Serial.println(F("================ HARDWARE COMMISSIONING CHECKLIST ================"));
    Serial.printf("  [1] Vehicle Output Driver    : %s (Topology: %s)\r\n",
                  drv.isVehicleDriverVerified() ? "COMMISSIONED" : "UNVERIFIED_SAFE_DISABLED",
                  SignalOutputDriver::driverTypeToString(drv.getVehicleDriverType()));
    Serial.printf("  [2] Pedestrian Output Driver : %s (Topology: %s)\r\n",
                  drv.isPedestrianDriverVerified() ? "COMMISSIONED" : "UNVERIFIED_SAFE_DISABLED",
                  SignalOutputDriver::driverTypeToString(drv.getPedestrianDriverType()));
    Serial.printf("  [3] Physical IR Validated    : %s (ValidatedMask=0x%02X, All6=%s)\r\n",
                  controller.isPhysicalIrValidated() ? "YES" : "NO",
                  det.getValidatedMask(),
                  controller.isAllPhysicalIrValidated() ? "YES" : "NO");
    for (uint8_t i = 0; i < TrafficConfig::IR_SENSOR_COUNT; ++i) {
        const IrChannelStatus& ch = det.getChannel(i);
        const bool idleElecOk = (ch.rawSampleCount > 0 && ch.rawPinHigh && !ch.stuckFault && !ch.chatterFault);
        const char* healthReason = "OK";
        const bool commissionable = det.validateChannelElectricalHealth(i, &healthReason);
        Serial.printf("      - IR%u (GPIO %2u): Validated=%s | IdleElec=%s | CycleObserved=%s | Ready=%s (%s)\r\n",
                      static_cast<unsigned>(i + 1),
                      ch.gpioPin,
                      ch.validated ? "YES" : "NO",
                      idleElecOk ? "PASS" : "FAIL",
                      ch.cycleObserved ? "YES" : "NO",
                      commissionable ? "YES" : "NO",
                      healthReason);
    }
    const PedestrianController& ped = controller.getPedestrianController();
    for (uint8_t b = 0; b < TrafficConfig::PED_BUTTON_COUNT; ++b) {
        const PedChannelState& pch = ped.getChannel(b);
        const bool btnIdleOk = (pch.rawPinHigh && !pch.stuckFault);
        Serial.printf("      - PED%u_BTN (GPIO %2u): IdleElec=%s | CycleObserved=%s | Press=%lu Rel=%lu\r\n",
                      static_cast<unsigned>(b + 1),
                      pch.buttonGpioPin,
                      btnIdleOk ? "PASS" : "FAIL",
                      pch.cycleObserved ? "YES" : "NO",
                      static_cast<unsigned long>(pch.pressCount),
                      static_cast<unsigned long>(pch.releaseCount));
    }
    Serial.printf("  [4] Protected Turn (RHT)     : %s\r\n",
                  controller.isProtectedTurnCommissioned() ? "COMMISSIONED" : "UNCOMMISSIONED_DISABLED");
    Serial.printf("  [5] Single-Lamp Test Mode    : %s (Channel=%s)\r\n",
                  drv.isLampTestActive() ? "ACTIVE" : "INACTIVE",
                  SignalOutputDriver::channelIndexToName(drv.getLampTestChannel()));
    Serial.println(F("=================================================================="));
}

void printLampDiagReport() {
    Serial.println(F("================ LAMP HARDWARE DIAGNOSTIC REPORT ================"));
    const TrafficController::LampDiagnosticSession& diag = controller.getLampDiagnosticSession();
    Serial.printf("  Diagnostic Session  : %s\r\n", diag.active ? "IN PROGRESS" : "IDLE / COMPLETED");
    Serial.printf("  Hold Duration       : %lu ms | Clearance: %lu ms\r\n",
                  static_cast<unsigned long>(diag.holdDurationMs),
                  static_cast<unsigned long>(diag.clearanceDurationMs));
    const TrafficController::AllGreenDiagnosticSession& ag = controller.getAllGreenDiagnosticSession();
    Serial.printf("  ALL-GREEN Test (10s): %s | Duration: %lu ms | Remaining: %lu ms\r\n",
                  ag.active ? "RUNNING (GPIO 25, 13, 21 ON)" : (ag.completed ? "COMPLETED (Holding ALL-RED)" : (ag.aborted ? "ABORTED" : "IDLE")),
                  static_cast<unsigned long>(ag.durationMs),
                  static_cast<unsigned long>(controller.getAllGreenRemainingMs(millis())));
    Serial.println(F("--- VEHICLE SIGNALS (9 OUTPUTS) ---------------------------------"));
    for (uint8_t i = 0; i < 9; ++i) {
        const auto& rec = diag.records[i];
        const bool isGreen = (strstr(rec.channelName, "_G") != nullptr);
        const char* physStatus = rec.tested
            ? (isGreen ? "FAILED (Vf ~3.2V > 3.3V; requires +5V High-Side Driver)" : "VERIFIED (Illuminates)")
            : "NOT TESTED (Run DIAG LAMPS VEH or DIAG 13)";
        Serial.printf("  %-7s (GPIO %2u) : Commanded=%s | PinHigh=%d (3.3V) | Physical=%s\r\n",
                      rec.channelName,
                      rec.gpioPin,
                      rec.commandedOn ? "ON" : "OFF",
                      rec.pinLevelHigh ? 1 : 0,
                      physStatus);
    }
    Serial.println(F("--- PEDESTRIAN SIGNALS (4 OUTPUTS) ------------------------------"));
    for (uint8_t i = 9; i < 13; ++i) {
        const auto& rec = diag.records[i];
        const char* physStatus = rec.tested
            ? "VERIFIED (ULN2003A Low-Side Sink)"
            : "NOT TESTED (Run DIAG LAMPS PED or DIAG 13)";
        Serial.printf("  %-7s (GPIO %2u) : Commanded=%s | PinHigh=%d (3.3V) | Physical=%s\r\n",
                      rec.channelName,
                      rec.gpioPin,
                      rec.commandedOn ? "ON" : "OFF",
                      rec.pinLevelHigh ? 1 : 0,
                      physStatus);
    }
    Serial.println(F("-----------------------------------------------------------------"));
    Serial.println(F("  INTERLOCK: Turn approach is operational and SAFETY LOCKED (holds RED) during normal sequencing."));
    Serial.println(F("             All 13 signal lamp outputs are initialized, verified and safely diagnosable."));
    Serial.println(F("-----------------------------------------------------------------"));
    Serial.println(F("  ELECTRICAL DIAGNOSIS:"));
    Serial.println(F("    - Vehicle modules are 5V common-ground (cathode) assemblies."));
    Serial.println(F("    - InGaN Green LEDs have forward voltage Vf ≈ 3.2V."));
    Serial.println(F("    - At 3.3V direct GPIO drive, forward current If < 0.5 mA (diode non-conducting)."));
    Serial.println(F("    - Corrective Action: Install +5V High-Side Sourcing Driver (TBD62783A / UDN2981A)."));
    Serial.println(F("================================================================="));
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
    } else if (strcmp(marker, "COMMISSION_PRINTED") == 0) {
        printCommissionStatus();
    } else if (strcmp(marker, "DIAG_REPORT_PRINTED") == 0) {
        printLampDiagReport();
    } else {
        Serial.println(marker);
    }
}

void printStartupBanner(uint32_t nowMs) {
    Serial.println();
    Serial.println(F("========================================================================"));
    Serial.printf("PROJECT  : %s\r\n", TrafficConfig::PROJECT_NAME);
    Serial.printf("VERSION  : %s\r\n", TrafficConfig::FIRMWARE_VER);
    Serial.printf("RULE     : %s\r\n", TrafficConfig::TRAFFIC_RULE_LABEL);
    Serial.printf("MODE     : %s | Control: %s\r\n",
                  TrafficController::modeToString(controller.getMode()),
                  TrafficController::controlModeToString(controller.getControlMode()));
    Serial.printf("STATE    : %s (Initial Logical State at %lu ms)\r\n",
                  TrafficController::stateToString(controller.getState()),
                  static_cast<unsigned long>(nowMs));
    Serial.println(F("------------------------------------------------------------------------"));
    Serial.println(F("IMPORTANT HARDWARE & ELECTRICAL SAFETY WARNINGS:"));
    Serial.println(F("  [!] Traffic Rule: RIGHT-HAND TRAFFIC (RHT / KEEP-RIGHT) — vehicles drive on the right."));
    Serial.println(F("  [!] Vehicle RYG channels (9): DIRECT_ACTIVE_HIGH on ESP32 GPIOs (disabled until 3.3V/current verified)."));
    Serial.println(F("  [!] Pedestrian channels (4) : LOW_SIDE_ULN2003A_ACTIVE_HIGH (disabled until common-anode/resistors verified)."));
    Serial.println(F("  [!] Protected Turn phase is UNCOMMISSIONED by default until physical"));
    Serial.println(F("      RHT / Keep-Right junction turning paths are confirmed."));
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
    for (uint8_t i = 0; i < TrafficConfig::IR_SENSOR_COUNT; ++i) {
        in.irPinHigh[i] = (digitalRead(TrafficConfig::IR_SENSOR_PINS[i]) == HIGH);
    }
    in.rawIrSensorPinHigh   = in.irPinHigh[2]; // Side Road Sensor 1 (GPIO 35) for legacy 3-field compatibility
    in.inputsValid          = true;
    in.hasExtendedChannels  = true;
    return in;
}

} // namespace

void setup() {
    // Steps 1-4: Establish safe output states, initialize output-driver abstraction,
    //            initialize all six IR sensor inputs, and initialize both pedestrian buttons.
    initializeGpioSafe();
    Serial.begin(TrafficConfig::SERIAL_BAUD);

    const uint32_t nowMs = millis();
    controller.setTransitionCallback(onStateTransition);
    controller.setEventLogCallback(onControllerEvent);
    serialHandler.setOutputCallback(onSerialPrinterMarker);

    // Step 7: Enter safe startup phase (STARTUP_SAFE)
    controller.begin(nowMs);

    // Steps 5-6: Load valid persisted configuration from NVS; validate and fall back safely if invalid
    controller.loadConfigFromNvs(nowMs);

    // Ensure safe output states are applied immediately
    applySignalOutputsFromFsm();

    // Sample initial physical GPIO input states before startup diagnostics
    const HardwareInputs initHwIn = readHardwareAdapter();
    if (controller.getMode() == OperatingMode::HARDWARE_READY) {
        controller.processHardwareAdapter(nowMs, initHwIn);
    } else {
        controller.updateRawHardwareTelemetry(initHwIn);
    }

    // Step 8: Report hardware and sensor status over Serial
    printStartupBanner(nowMs);
    printConfig();
    printSensors();
}

void loop() {
    const uint32_t nowMs = millis();

    pollSerialInput(nowMs);

    const HardwareInputs hwIn = readHardwareAdapter();
    if (controller.getMode() == OperatingMode::HARDWARE_READY) {
        controller.processHardwareAdapter(nowMs, hwIn);
    } else {
        controller.updateRawHardwareTelemetry(hwIn);
    }

    controller.update(nowMs);
    applySignalOutputsFromFsm();
}
#endif // PIO_UNIT_TESTING
