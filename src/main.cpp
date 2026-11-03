#include <Arduino.h>
#include <string.h>
#include <ctype.h>
#include "TrafficConfig.h"
#include "TrafficController.h"

#ifndef PIO_UNIT_TESTING
namespace {

TrafficController controller;
TrafficState lastAppliedState = TrafficState::STARTUP_SAFE;
bool firstOutputPass = true;

constexpr size_t CMD_BUFFER_SIZE = 64;
char cmdBuffer[CMD_BUFFER_SIZE];
size_t cmdIndex = 0;

// ============================================================================
// CENTRALIZED GPIO INITIALIZATION & OUTPUT MANAGEMENT
// ============================================================================
void initializeGpioSafe() {
    // 1. Initialize output latches to LOW BEFORE switching pins to OUTPUT mode
    // so external outputs are never momentarily energized during startup.
    const uint8_t outputPins[] = {
        TrafficConfig::PIN_MAIN_GREEN,
        TrafficConfig::PIN_MAIN_YELLOW,
        TrafficConfig::PIN_MAIN_RED,
        TrafficConfig::PIN_SIDE_GREEN,
        TrafficConfig::PIN_SIDE_YELLOW,
        TrafficConfig::PIN_SIDE_RED,
        TrafficConfig::PIN_PED_WALK,
        TrafficConfig::PIN_PED_DONT_WALK
    };

    for (uint8_t pin : outputPins) {
        digitalWrite(pin, LOW);
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }

    // 2. Configure physical sensor and button inputs with INPUT_PULLUP
    pinMode(TrafficConfig::PIN_SIDE_IR_SENSOR, INPUT_PULLUP);
    pinMode(TrafficConfig::PIN_PED_BUTTON, INPUT_PULLUP);
}

// Single centralized function that derives and writes all signal outputs from current FSM state
void applySignalOutputsFromFsm() {
    const TrafficState currentState = controller.getState();
    const SignalOutputs out = controller.getOutputs();
    const char* violation = nullptr;

    if (!TrafficController::verifySafetyInvariants(currentState, out, &violation)) {
        // Force all outputs LOW immediately and enter FAULT_SAFE
        digitalWrite(TrafficConfig::PIN_MAIN_GREEN, LOW);
        digitalWrite(TrafficConfig::PIN_MAIN_YELLOW, LOW);
        digitalWrite(TrafficConfig::PIN_MAIN_RED, LOW);
        digitalWrite(TrafficConfig::PIN_SIDE_GREEN, LOW);
        digitalWrite(TrafficConfig::PIN_SIDE_YELLOW, LOW);
        digitalWrite(TrafficConfig::PIN_SIDE_RED, LOW);
        digitalWrite(TrafficConfig::PIN_PED_WALK, LOW);
        digitalWrite(TrafficConfig::PIN_PED_DONT_WALK, LOW);
        controller.triggerFault(millis(), violation ? violation : "OUTPUT_SAFETY_CHECK_FAILED");
        return;
    }

    if (!firstOutputPass && currentState == lastAppliedState) {
        return;
    }

    // Safe turn-off-before-turn-on sequencing when changing states
    if (!out.mainGreen)   digitalWrite(TrafficConfig::PIN_MAIN_GREEN, LOW);
    if (!out.mainYellow)  digitalWrite(TrafficConfig::PIN_MAIN_YELLOW, LOW);
    if (!out.mainRed)     digitalWrite(TrafficConfig::PIN_MAIN_RED, LOW);
    if (!out.sideGreen)   digitalWrite(TrafficConfig::PIN_SIDE_GREEN, LOW);
    if (!out.sideYellow)  digitalWrite(TrafficConfig::PIN_SIDE_YELLOW, LOW);
    if (!out.sideRed)     digitalWrite(TrafficConfig::PIN_SIDE_RED, LOW);
    if (!out.pedWalk)     digitalWrite(TrafficConfig::PIN_PED_WALK, LOW);
    if (!out.pedDontWalk) digitalWrite(TrafficConfig::PIN_PED_DONT_WALK, LOW);

    if (out.mainGreen)    digitalWrite(TrafficConfig::PIN_MAIN_GREEN, HIGH);
    if (out.mainYellow)   digitalWrite(TrafficConfig::PIN_MAIN_YELLOW, HIGH);
    if (out.mainRed)      digitalWrite(TrafficConfig::PIN_MAIN_RED, HIGH);
    if (out.sideGreen)    digitalWrite(TrafficConfig::PIN_SIDE_GREEN, HIGH);
    if (out.sideYellow)   digitalWrite(TrafficConfig::PIN_SIDE_YELLOW, HIGH);
    if (out.sideRed)      digitalWrite(TrafficConfig::PIN_SIDE_RED, HIGH);
    if (out.pedWalk)      digitalWrite(TrafficConfig::PIN_PED_WALK, HIGH);
    if (out.pedDontWalk)  digitalWrite(TrafficConfig::PIN_PED_DONT_WALK, HIGH);

    lastAppliedState = currentState;
    firstOutputPass  = false;
}

// ============================================================================
// DIAGNOSTICS & LOGGING CALLBACKS
// ============================================================================
void onStateTransition(uint32_t nowMs, TrafficState from, TrafficState to, const char* reason) {
    Serial.printf("[%lu ms] %s -> %s | Reason: %s\r\n",
                  static_cast<unsigned long>(nowMs),
                  TrafficController::stateToString(from),
                  TrafficController::stateToString(to),
                  reason ? reason : "NONE");

    if (to == TrafficState::FAULT_SAFE) {
        Serial.println(F("[FAULT_SAFE] All configured GPIO outputs commanded LOW."));
        Serial.println(F("[WARNING] Physical fail-safe behavior is NOT guaranteed by software alone."));
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
    Serial.println(F("  HELP         - List supported commands"));
    Serial.println(F("  STATUS       - Print current state, mode, timers, and pending requests"));
    Serial.println(F("  SIM ON       - Enable simulated demand inputs (SERIAL_SIMULATION mode)"));
    Serial.println(F("  SIM OFF      - Disable simulation and return to HARDWARE_READY mode"));
    Serial.println(F("  VEHICLE ON   - Simulate a side-road vehicle waiting (SIM mode only)"));
    Serial.println(F("  VEHICLE OFF  - Clear simulated side-road vehicle demand (SIM mode only)"));
    Serial.println(F("  PEDESTRIAN   - Register a simulated pedestrian request (SIM mode only)"));
    Serial.println(F("  CLEAR        - Clear pending simulated demand only (SIM mode only)"));
    Serial.println(F("  RESET        - Perform controlled logical reset without MCU restart"));
    Serial.println(F("------------------------------------------------------------------------"));
}

void printStatus(uint32_t nowMs) {
    const TrafficState st = controller.getState();
    const SignalOutputs out = controller.getOutputs();
    Serial.println(F("================ CONTROLLER STATUS ================"));
    Serial.printf("  Timestamp         : %lu ms\r\n", static_cast<unsigned long>(nowMs));
    Serial.printf("  Operating Mode    : %s\r\n", TrafficController::modeToString(controller.getMode()));
    Serial.printf("  Physical IR Valid : %s (Hardware unverified in bench test)\r\n",
                  controller.isPhysicalIrValidated() ? "YES" : "NO");
    Serial.printf("  Current FSM State : %s\r\n", TrafficController::stateToString(st));
    Serial.printf("  State Elapsed     : %lu ms\r\n", static_cast<unsigned long>(controller.getStateElapsed(nowMs)));
    if (st == TrafficState::SIDE_GREEN) {
        Serial.printf("  Side Gap Timer    : %s (%lu ms)\r\n",
                      controller.isSideGapTimerActive() ? "ACTIVE" : "INACTIVE",
                      static_cast<unsigned long>(controller.getSideGapElapsed(nowMs)));
    }
    Serial.printf("  Next Target Phase : %s\r\n", TrafficController::targetToString(controller.getNextTarget()));
    Serial.printf("  Side Demand       : %s (SimVeh=%s, HwLatched=%s)\r\n",
                  controller.hasSideVehicleDemand() ? "PENDING" : "NONE",
                  controller.isSimVehicleActive() ? "ON" : "OFF",
                  controller.isHwVehicleLatched() ? "YES" : "NO");
    Serial.printf("  Pedestrian Demand : %s\r\n", controller.hasPedestrianDemand() ? "LATCHED" : "NONE");
    Serial.printf("  Logical Outputs   : MAIN[G=%d Y=%d R=%d] SIDE[G=%d Y=%d R=%d] PED[WALK=%d DONT_WALK=%d]\r\n",
                  out.mainGreen, out.mainYellow, out.mainRed,
                  out.sideGreen, out.sideYellow, out.sideRed,
                  out.pedWalk, out.pedDontWalk);
    if (st == TrafficState::FAULT_SAFE) {
        Serial.printf("  Fault Reason      : %s\r\n", controller.getFaultReason());
    }
    Serial.println(F("==================================================="));
}

void printStartupBanner(uint32_t nowMs) {
    Serial.println();
    Serial.println(F("========================================================================"));
    Serial.printf("PROJECT  : %s\r\n", TrafficConfig::PROJECT_NAME);
    Serial.printf("VERSION  : %s\r\n", TrafficConfig::FIRMWARE_VER);
    Serial.printf("MODE     : %s (Default Startup Mode)\r\n",
                  TrafficController::modeToString(controller.getMode()));
    Serial.printf("STATE    : %s (Initial Logical State at %lu ms)\r\n",
                  TrafficController::stateToString(controller.getState()),
                  static_cast<unsigned long>(nowMs));
    Serial.println(F("------------------------------------------------------------------------"));
    Serial.println(F("GPIO CONFIGURATION SUMMARY:"));
    Serial.printf("  Main Road  (OUTPUT)       : GREEN=%u, YELLOW=%u, RED=%u (Init LOW)\r\n",
                  TrafficConfig::PIN_MAIN_GREEN, TrafficConfig::PIN_MAIN_YELLOW, TrafficConfig::PIN_MAIN_RED);
    Serial.printf("  Side Road  (OUTPUT)       : GREEN=%u, YELLOW=%u, RED=%u (Init LOW)\r\n",
                  TrafficConfig::PIN_SIDE_GREEN, TrafficConfig::PIN_SIDE_YELLOW, TrafficConfig::PIN_SIDE_RED);
    Serial.printf("  Pedestrian (OUTPUT)       : WALK=%u, DON'T WALK=%u (Init LOW)\r\n",
                  TrafficConfig::PIN_PED_WALK, TrafficConfig::PIN_PED_DONT_WALK);
    Serial.printf("  Inputs     (INPUT_PULLUP) : Side IR=%u (Active LOW), Ped Btn=%u (Active LOW, %lu ms debounce)\r\n",
                  TrafficConfig::PIN_SIDE_IR_SENSOR, TrafficConfig::PIN_PED_BUTTON,
                  static_cast<unsigned long>(TrafficConfig::PED_DEBOUNCE_MS));
    Serial.println(F("------------------------------------------------------------------------"));
    Serial.println(F("BENCH-TEST DEMONSTRATION TIMINGS (Not certified for public-road use):"));
    Serial.printf("  Startup Safe   : %lu ms | All-Red Clearance : %lu ms\r\n",
                  static_cast<unsigned long>(TrafficConfig::STARTUP_SAFE_MS),
                  static_cast<unsigned long>(TrafficConfig::ALL_RED_MS));
    Serial.printf("  Main Min Green : %lu ms | Main Yellow       : %lu ms\r\n",
                  static_cast<unsigned long>(TrafficConfig::MAIN_MIN_GREEN_MS),
                  static_cast<unsigned long>(TrafficConfig::MAIN_YELLOW_MS));
    Serial.printf("  Side Min Green : %lu ms | Side Max Green    : %lu ms\r\n",
                  static_cast<unsigned long>(TrafficConfig::SIDE_MIN_GREEN_MS),
                  static_cast<unsigned long>(TrafficConfig::SIDE_MAX_GREEN_MS));
    Serial.printf("  Side Clear Gap : %lu ms | Side Yellow       : %lu ms\r\n",
                  static_cast<unsigned long>(TrafficConfig::SIDE_CLEAR_GAP_MS),
                  static_cast<unsigned long>(TrafficConfig::SIDE_YELLOW_MS));
    Serial.printf("  Ped WALK Phase : %lu ms\r\n",
                  static_cast<unsigned long>(TrafficConfig::PED_WALK_MS));
    Serial.println(F("------------------------------------------------------------------------"));
    Serial.println(F("IMPORTANT HARDWARE & SAFETY WARNINGS:"));
    Serial.println(F("  [!] BENCH-TEST FIRMWARE ONLY: No physical LEDs, IR sensors, or buttons"));
    Serial.println(F("      are currently verified/connected. Initial ALL_RED is logical only."));
    Serial.println(F("  [!] Physical input circuitry on GPIO 32 & GPIO 33 has NOT been validated."));
    Serial.println(F("      INPUT_PULLUP HIGH readings do NOT prove sensor connection or health."));
    Serial.println(F("  [!] Do NOT treat this controller as a certified physical safety system."));
    printHelp();
}

// ============================================================================
// NON-BLOCKING SERIAL COMMAND PARSER
// ============================================================================
void handleSerialCommand(uint32_t nowMs, char* rawLine) {
    // Trim leading whitespace
    while (*rawLine && isspace(static_cast<unsigned char>(*rawLine))) {
        rawLine++;
    }
    // Trim trailing whitespace
    size_t len = strlen(rawLine);
    while (len > 0 && isspace(static_cast<unsigned char>(rawLine[len - 1]))) {
        rawLine[len - 1] = '\0';
        len--;
    }
    if (len == 0) {
        return;
    }

    // Normalize to uppercase for case-insensitive matching
    for (size_t i = 0; i < len; ++i) {
        rawLine[i] = static_cast<char>(toupper(static_cast<unsigned char>(rawLine[i])));
    }

    if (strcmp(rawLine, "HELP") == 0) {
        printHelp();
    } else if (strcmp(rawLine, "STATUS") == 0) {
        printStatus(nowMs);
    } else if (strcmp(rawLine, "SIM ON") == 0) {
        controller.setSimulationMode(nowMs, true);
    } else if (strcmp(rawLine, "SIM OFF") == 0) {
        controller.setSimulationMode(nowMs, false);
        applySignalOutputsFromFsm();
    } else if (strcmp(rawLine, "RESET") == 0) {
        controller.resetLogical(nowMs, "SERIAL_RESET_COMMAND");
        applySignalOutputsFromFsm();
    } else if (strcmp(rawLine, "VEHICLE ON") == 0) {
        controller.setSimVehicleDemand(nowMs, true);
    } else if (strcmp(rawLine, "VEHICLE OFF") == 0) {
        controller.setSimVehicleDemand(nowMs, false);
    } else if (strcmp(rawLine, "PEDESTRIAN") == 0) {
        controller.triggerSimPedestrianRequest(nowMs);
    } else if (strcmp(rawLine, "CLEAR") == 0) {
        controller.clearSimulatedDemand(nowMs);
    } else {
        Serial.printf("[%lu ms] EVENT: COMMAND_REJECTED | Unknown or unsupported command: '%s' (Type HELP)\r\n",
                      static_cast<unsigned long>(nowMs), rawLine);
    }
}

void pollSerialInput(uint32_t nowMs) {
    while (Serial.available() > 0) {
        const char c = static_cast<char>(Serial.read());
        if (c == '\r' || c == '\n') {
            if (cmdIndex > 0) {
                cmdBuffer[cmdIndex] = '\0';
                handleSerialCommand(nowMs, cmdBuffer);
                cmdIndex = 0;
            }
        } else if (cmdIndex < (CMD_BUFFER_SIZE - 1)) {
            cmdBuffer[cmdIndex++] = c;
        } else {
            // Buffer overflow protection: discard line and notify
            cmdIndex = 0;
            Serial.printf("[%lu ms] EVENT: COMMAND_REJECTED | Serial command too long\r\n",
                          static_cast<unsigned long>(nowMs));
        }
    }
}

// Hardware Input Adapter: isolates physical GPIO reads from the TrafficController FSM
HardwareInputs readHardwareAdapter() {
    HardwareInputs in;
    in.rawPedButtonPinHigh = (digitalRead(TrafficConfig::PIN_PED_BUTTON) == HIGH);
    in.rawIrSensorPinHigh  = (digitalRead(TrafficConfig::PIN_SIDE_IR_SENSOR) == HIGH);
    in.inputsValid         = true;
    return in;
}

} // namespace

void setup() {
    // 1. Configure output pins to inactive LOW state before OUTPUT mode, and configure inputs
    initializeGpioSafe();

    // 2. Initialize Serial interface
    Serial.begin(TrafficConfig::SERIAL_BAUD);

    // 3. Initialize state timers, demand flags, and callbacks; enter STARTUP_SAFE
    const uint32_t nowMs = millis();
    controller.setTransitionCallback(onStateTransition);
    controller.setEventLogCallback(onControllerEvent);
    controller.begin(nowMs);

    // 4. Ensure outputs match STARTUP_SAFE (all LOW)
    applySignalOutputsFromFsm();

    // 5. Print startup diagnostic banner
    printStartupBanner(nowMs);
}

void loop() {
    const uint32_t nowMs = millis();

    // 1. Poll Serial Monitor commands (non-blocking)
    pollSerialInput(nowMs);

    // 2. Read and filter physical inputs via HardwareInputs adapter when in HARDWARE_READY mode
    if (controller.getMode() == OperatingMode::HARDWARE_READY) {
        const HardwareInputs hwIn = readHardwareAdapter();
        controller.processHardwareAdapter(nowMs, hwIn);
    }

    // 3. Advance FSM timers and phase transitions (non-blocking, rollover-safe)
    controller.update(nowMs);

    // 4. Centralize signal output management derived strictly from current FSM state
    applySignalOutputsFromFsm();
}
#endif // PIO_UNIT_TESTING

