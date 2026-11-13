#ifndef SERIAL_COMMAND_HANDLER_H
#define SERIAL_COMMAND_HANDLER_H

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "TrafficController.h"

enum class SerialCommandResult : uint8_t {
    OK_EXECUTED         = 0,
    EMPTY_IGNORED       = 1,
    REJECTED_BY_STATE   = 2,
    UNKNOWN_INVALID_CMD = 3
};

struct DashboardStatusSnapshot {
    uint32_t        timestampMs;
    bool            rightHandTrafficRule;
    OperatingMode   operatingMode;
    ControlMode     controlMode;
    TrafficState    currentFsmState;
    uint32_t        stateElapsedMs;
    bool            sideGapActive;
    uint32_t        sideGapElapsedMs;
    NextPhaseTarget nextTargetPhase;
    bool            protectedTurnCommissioned;
    bool            mainDemand;
    bool            sideDemand;
    bool            turnDemand;
    bool            ped1Demand;
    bool            ped2Demand;
    bool            pedClearanceActive;
    bool            physicalIrValid;
    uint8_t         validatedIrMask;
    SignalOutputs   outputs;
    SignalOutputs   electricalOutputs;
    uint16_t        activeFaultMask;
    bool            valid;
};

class SerialCommandHandler {
public:
    typedef void (*OutputPrintCallback)(const char* line);

    SerialCommandHandler() : printCb_(nullptr), rebootRequested_(false) {}

    void setOutputCallback(OutputPrintCallback cb) { printCb_ = cb; }
    bool isRebootRequested() const { return rebootRequested_; }
    void clearRebootRequest() { rebootRequested_ = false; }

    SerialCommandResult executeCommand(uint32_t nowMs, const char* inputLine, TrafficController& ctrl) {
        if (!inputLine) return SerialCommandResult::UNKNOWN_INVALID_CMD;

        char buf[128];
        strncpy(buf, inputLine, sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = '\0';

        // Trim leading whitespace
        char* cmd = buf;
        while (*cmd && isspace(static_cast<unsigned char>(*cmd))) {
            cmd++;
        }
        // Trim trailing whitespace
        size_t len = strlen(cmd);
        while (len > 0 && isspace(static_cast<unsigned char>(cmd[len - 1]))) {
            cmd[len - 1] = '\0';
            len--;
        }
        if (len == 0) {
            return SerialCommandResult::EMPTY_IGNORED;
        }

        // Convert to uppercase and collapse internal multiple spaces to single spaces
        char normalized[128];
        size_t outIdx = 0;
        bool prevSpace = false;
        for (size_t i = 0; i < len && outIdx < (sizeof(normalized) - 1); ++i) {
            const unsigned char ch = static_cast<unsigned char>(cmd[i]);
            if (isspace(ch)) {
                if (!prevSpace) {
                    normalized[outIdx++] = ' ';
                    prevSpace = true;
                }
            } else {
                normalized[outIdx++] = static_cast<char>(toupper(ch));
                prevSpace = false;
            }
        }
        normalized[outIdx] = '\0';

        if (strcmp(normalized, "HELP") == 0) {
            emitLine("HELP_PRINTED");
            return SerialCommandResult::OK_EXECUTED;
        }
        if (strcmp(normalized, "STATUS") == 0) {
            emitLine("STATUS_PRINTED");
            return SerialCommandResult::OK_EXECUTED;
        }
        if (strcmp(normalized, "SENSORS") == 0) {
            emitLine("SENSORS_PRINTED");
            return SerialCommandResult::OK_EXECUTED;
        }
        if (strcmp(normalized, "DEMAND") == 0) {
            emitLine("DEMAND_PRINTED");
            return SerialCommandResult::OK_EXECUTED;
        }
        if (strcmp(normalized, "PHASES") == 0) {
            emitLine("PHASES_PRINTED");
            return SerialCommandResult::OK_EXECUTED;
        }
        if (strcmp(normalized, "TIMING") == 0) {
            emitLine("TIMING_PRINTED");
            return SerialCommandResult::OK_EXECUTED;
        }
        if (strcmp(normalized, "TIMING PROTOTYPE") == 0 || strcmp(normalized, "TIMING FIXED") == 0) {
            ctrl.setTimingConfig(nowMs, TrafficConfig::PROTOTYPE_TIMING_CONFIG);
            const bool saved = ctrl.saveConfigToNvs(nowMs);
            emitLine(saved ? "TIMING_PROTOTYPE_APPLIED: NVS_OK" : "TIMING_PROTOTYPE_APPLIED: NVS_FAIL");
            return SerialCommandResult::OK_EXECUTED;
        }
        if (strcmp(normalized, "TIMING BENCH") == 0) {
            ctrl.setTimingConfig(nowMs, TrafficConfig::BENCH_TIMING_CONFIG);
            const bool saved = ctrl.saveConfigToNvs(nowMs);
            emitLine(saved ? "TIMING_BENCH_APPLIED: NVS_OK" : "TIMING_BENCH_APPLIED: NVS_FAIL");
            return SerialCommandResult::OK_EXECUTED;
        }
        if (strncmp(normalized, "TIMING SET ", 11) == 0) {
            unsigned long mMin = 0, mMax = 0, mYel = 0;
            unsigned long sMin = 0, sMax = 0, sYel = 0;
            unsigned long tMin = 0, tMax = 0, tYel = 0;
            unsigned long pWalk = 0, pClear = 0, ar = 0;
            const int n = sscanf(normalized + 11, "%lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu",
                                 &mMin, &mMax, &mYel,
                                 &sMin, &sMax, &sYel,
                                 &tMin, &tMax, &tYel,
                                 &pWalk, &pClear, &ar);
            if (n < 12) {
                emitLine("TIMING_REJECTED: Expected 12 arguments: <mMin> <mMax> <mYel> <sMin> <sMax> <sYel> <tMin> <tMax> <tYel> <pWalk> <pClear> <allRed>");
                return SerialCommandResult::UNKNOWN_INVALID_CMD;
            }
            if (mMin < TrafficConfig::MIN_ALLOWED_INTERVAL_MS || mMin > TrafficConfig::MAX_ALLOWED_INTERVAL_MS ||
                mMax < mMin || mMax > TrafficConfig::MAX_ALLOWED_INTERVAL_MS) {
                emitLine("TIMING_REJECTED: Main Green bounds invalid (Min>=500, Max>=Min, Max<=120000)");
                return SerialCommandResult::REJECTED_BY_STATE;
            }
            if (sMin < TrafficConfig::MIN_ALLOWED_INTERVAL_MS || sMin > TrafficConfig::MAX_ALLOWED_INTERVAL_MS ||
                sMax < sMin || sMax > TrafficConfig::MAX_ALLOWED_INTERVAL_MS) {
                emitLine("TIMING_REJECTED: Side Green bounds invalid (Min>=500, Max>=Min, Max<=120000)");
                return SerialCommandResult::REJECTED_BY_STATE;
            }
            if (tMin < TrafficConfig::MIN_ALLOWED_INTERVAL_MS || tMin > TrafficConfig::MAX_ALLOWED_INTERVAL_MS ||
                tMax < tMin || tMax > TrafficConfig::MAX_ALLOWED_INTERVAL_MS) {
                emitLine("TIMING_REJECTED: Turn Green bounds invalid (Min>=500, Max>=Min, Max<=120000)");
                return SerialCommandResult::REJECTED_BY_STATE;
            }
            if (mYel < 1000 || mYel > TrafficConfig::MAX_ALLOWED_INTERVAL_MS ||
                sYel < 1000 || sYel > TrafficConfig::MAX_ALLOWED_INTERVAL_MS ||
                tYel < 1000 || tYel > TrafficConfig::MAX_ALLOWED_INTERVAL_MS) {
                emitLine("TIMING_REJECTED: Yellow clearance must be >= 1000 ms");
                return SerialCommandResult::REJECTED_BY_STATE;
            }
            if (ar < 1000 || ar > TrafficConfig::MAX_ALLOWED_INTERVAL_MS) {
                emitLine("TIMING_REJECTED: All-Red clearance must be >= 1000 ms");
                return SerialCommandResult::REJECTED_BY_STATE;
            }
            if (pWalk < 1000 || pWalk > TrafficConfig::MAX_ALLOWED_INTERVAL_MS ||
                pClear < 1000 || pClear > TrafficConfig::MAX_ALLOWED_INTERVAL_MS) {
                emitLine("TIMING_REJECTED: Pedestrian walk/clearance must be >= 1000 ms");
                return SerialCommandResult::REJECTED_BY_STATE;
            }

            TrafficConfig::TimingConfig tc = ctrl.getTimingConfig();
            tc.mainMinGreenMs = mMin;
            tc.mainMaxGreenMs = mMax;
            tc.mainYellowMs   = mYel;
            tc.sideMinGreenMs = sMin;
            tc.sideMaxGreenMs = sMax;
            tc.sideYellowMs   = sYel;
            tc.turnMinGreenMs = tMin;
            tc.turnMaxGreenMs = tMax;
            tc.turnYellowMs   = tYel;
            tc.pedWalkMs      = pWalk;
            tc.pedClearanceMs = pClear;
            tc.allRedMs       = ar;

            if (!ctrl.setTimingConfig(nowMs, tc)) {
                emitLine("TIMING_REJECTED: Controller validation failed");
                return SerialCommandResult::REJECTED_BY_STATE;
            }
            const bool saved = ctrl.saveConfigToNvs(nowMs);
            emitLine(saved ? "TIMING_APPLIED: NVS_OK" : "TIMING_APPLIED: NVS_FAIL");
            return SerialCommandResult::OK_EXECUTED;
        }
        if (strncmp(normalized, "TIMING MAIN ", 12) == 0) {
            unsigned long minG = 0, yel = 0, maxG = 0;
            const int n = sscanf(normalized + 12, "%lu %lu %lu", &minG, &yel, &maxG);
            if (n >= 1) {
                if (minG < TrafficConfig::MIN_ALLOWED_INTERVAL_MS || minG > TrafficConfig::MAX_ALLOWED_INTERVAL_MS) {
                    emitLine("TIMING_REJECTED: Main Min Green out of bounds (500..120000 ms)");
                    return SerialCommandResult::REJECTED_BY_STATE;
                }
                if (n >= 2 && yel > 0 && yel < 1000) {
                    emitLine("TIMING_REJECTED: Yellow clearance must be >= 1000 ms");
                    return SerialCommandResult::REJECTED_BY_STATE;
                }
                if (n >= 3 && maxG > 0 && maxG < minG) {
                    emitLine("TIMING_REJECTED: Main Max Green must be >= Min Green");
                    return SerialCommandResult::REJECTED_BY_STATE;
                }
                TrafficConfig::TimingConfig tc = ctrl.getTimingConfig();
                tc.mainMinGreenMs = minG;
                if (n >= 2 && yel > 0) tc.mainYellowMs = yel;
                if (n >= 3 && maxG > 0) {
                    tc.mainMaxGreenMs = maxG;
                } else if (tc.mainMaxGreenMs < tc.mainMinGreenMs) {
                    tc.mainMaxGreenMs = tc.mainMinGreenMs + 5000;
                }
                if (!ctrl.setTimingConfig(nowMs, tc)) {
                    emitLine("TIMING_REJECTED: Controller rejected timing configuration");
                    return SerialCommandResult::REJECTED_BY_STATE;
                }
                const bool saved = ctrl.saveConfigToNvs(nowMs);
                emitLine(saved ? "TIMING_APPLIED: NVS_OK" : "TIMING_APPLIED: NVS_FAIL");
                return SerialCommandResult::OK_EXECUTED;
            }
            emitLine("TIMING_REJECTED: Missing arguments for TIMING MAIN");
            return SerialCommandResult::UNKNOWN_INVALID_CMD;
        }
        if (strncmp(normalized, "TIMING SIDE ", 12) == 0) {
            unsigned long minG = 0, yel = 0, maxG = 0;
            const int n = sscanf(normalized + 12, "%lu %lu %lu", &minG, &yel, &maxG);
            if (n >= 1) {
                if (minG < TrafficConfig::MIN_ALLOWED_INTERVAL_MS || minG > TrafficConfig::MAX_ALLOWED_INTERVAL_MS) {
                    emitLine("TIMING_REJECTED: Side Min Green out of bounds (500..120000 ms)");
                    return SerialCommandResult::REJECTED_BY_STATE;
                }
                if (n >= 2 && yel > 0 && yel < 1000) {
                    emitLine("TIMING_REJECTED: Yellow clearance must be >= 1000 ms");
                    return SerialCommandResult::REJECTED_BY_STATE;
                }
                if (n >= 3 && maxG > 0 && maxG < minG) {
                    emitLine("TIMING_REJECTED: Side Max Green must be >= Min Green");
                    return SerialCommandResult::REJECTED_BY_STATE;
                }
                TrafficConfig::TimingConfig tc = ctrl.getTimingConfig();
                tc.sideMinGreenMs = minG;
                if (n >= 2 && yel > 0) tc.sideYellowMs = yel;
                if (n >= 3 && maxG > 0) {
                    tc.sideMaxGreenMs = maxG;
                } else if (tc.sideMaxGreenMs < tc.sideMinGreenMs) {
                    tc.sideMaxGreenMs = tc.sideMinGreenMs + 5000;
                }
                if (!ctrl.setTimingConfig(nowMs, tc)) {
                    emitLine("TIMING_REJECTED: Controller rejected timing configuration");
                    return SerialCommandResult::REJECTED_BY_STATE;
                }
                const bool saved = ctrl.saveConfigToNvs(nowMs);
                emitLine(saved ? "TIMING_APPLIED: NVS_OK" : "TIMING_APPLIED: NVS_FAIL");
                return SerialCommandResult::OK_EXECUTED;
            }
            emitLine("TIMING_REJECTED: Missing arguments for TIMING SIDE");
            return SerialCommandResult::UNKNOWN_INVALID_CMD;
        }
        if (strncmp(normalized, "TIMING TURN ", 12) == 0) {
            unsigned long minG = 0, yel = 0, maxG = 0;
            const int n = sscanf(normalized + 12, "%lu %lu %lu", &minG, &yel, &maxG);
            if (n >= 1) {
                if (minG < TrafficConfig::MIN_ALLOWED_INTERVAL_MS || minG > TrafficConfig::MAX_ALLOWED_INTERVAL_MS) {
                    emitLine("TIMING_REJECTED: Turn Min Green out of bounds (500..120000 ms)");
                    return SerialCommandResult::REJECTED_BY_STATE;
                }
                if (n >= 2 && yel > 0 && yel < 1000) {
                    emitLine("TIMING_REJECTED: Yellow clearance must be >= 1000 ms");
                    return SerialCommandResult::REJECTED_BY_STATE;
                }
                if (n >= 3 && maxG > 0 && maxG < minG) {
                    emitLine("TIMING_REJECTED: Turn Max Green must be >= Min Green");
                    return SerialCommandResult::REJECTED_BY_STATE;
                }
                TrafficConfig::TimingConfig tc = ctrl.getTimingConfig();
                tc.turnMinGreenMs = minG;
                if (n >= 2 && yel > 0) tc.turnYellowMs = yel;
                if (n >= 3 && maxG > 0) {
                    tc.turnMaxGreenMs = maxG;
                } else if (tc.turnMaxGreenMs < tc.turnMinGreenMs) {
                    tc.turnMaxGreenMs = tc.turnMinGreenMs + 4000;
                }
                if (!ctrl.setTimingConfig(nowMs, tc)) {
                    emitLine("TIMING_REJECTED: Controller rejected timing configuration");
                    return SerialCommandResult::REJECTED_BY_STATE;
                }
                const bool saved = ctrl.saveConfigToNvs(nowMs);
                emitLine(saved ? "TIMING_APPLIED: NVS_OK" : "TIMING_APPLIED: NVS_FAIL");
                return SerialCommandResult::OK_EXECUTED;
            }
            emitLine("TIMING_REJECTED: Missing arguments for TIMING TURN");
            return SerialCommandResult::UNKNOWN_INVALID_CMD;
        }
        if (strncmp(normalized, "TIMING ALL_RED ", 15) == 0 || strncmp(normalized, "TIMING ALLRED ", 14) == 0) {
            unsigned long ar = 0;
            const char* arg = (normalized[9] == 'R') ? (normalized + 14) : (normalized + 15);
            if (sscanf(arg, "%lu", &ar) == 1) {
                if (ar < 1000 || ar > TrafficConfig::MAX_ALLOWED_INTERVAL_MS) {
                    emitLine("TIMING_REJECTED: All-Red clearance must be >= 1000 ms and <= 120000 ms");
                    return SerialCommandResult::REJECTED_BY_STATE;
                }
                TrafficConfig::TimingConfig tc = ctrl.getTimingConfig();
                tc.allRedMs = ar;
                if (!ctrl.setTimingConfig(nowMs, tc)) {
                    emitLine("TIMING_REJECTED: Controller rejected timing configuration");
                    return SerialCommandResult::REJECTED_BY_STATE;
                }
                const bool saved = ctrl.saveConfigToNvs(nowMs);
                emitLine(saved ? "TIMING_APPLIED: NVS_OK" : "TIMING_APPLIED: NVS_FAIL");
                return SerialCommandResult::OK_EXECUTED;
            }
            emitLine("TIMING_REJECTED: Missing argument for TIMING ALL_RED");
            return SerialCommandResult::UNKNOWN_INVALID_CMD;
        }
        if (strncmp(normalized, "TIMING PED ", 11) == 0) {
            unsigned long walk = 0, clear = 0;
            const int n = sscanf(normalized + 11, "%lu %lu", &walk, &clear);
            if (n >= 1) {
                if (walk < 1000 || walk > TrafficConfig::MAX_ALLOWED_INTERVAL_MS) {
                    emitLine("TIMING_REJECTED: Pedestrian WALK must be >= 1000 ms");
                    return SerialCommandResult::REJECTED_BY_STATE;
                }
                if (n >= 2 && clear > 0 && clear < 1000) {
                    emitLine("TIMING_REJECTED: Pedestrian clearance must be >= 1000 ms");
                    return SerialCommandResult::REJECTED_BY_STATE;
                }
                TrafficConfig::TimingConfig tc = ctrl.getTimingConfig();
                tc.pedWalkMs = walk;
                if (n >= 2 && clear > 0) tc.pedClearanceMs = clear;
                if (!ctrl.setTimingConfig(nowMs, tc)) {
                    emitLine("TIMING_REJECTED: Controller rejected timing configuration");
                    return SerialCommandResult::REJECTED_BY_STATE;
                }
                const bool saved = ctrl.saveConfigToNvs(nowMs);
                emitLine(saved ? "TIMING_APPLIED: NVS_OK" : "TIMING_APPLIED: NVS_FAIL");
                return SerialCommandResult::OK_EXECUTED;
            }
            emitLine("TIMING_REJECTED: Missing arguments for TIMING PED");
            return SerialCommandResult::UNKNOWN_INVALID_CMD;
        }
        if (strcmp(normalized, "CONFIG") == 0) {
            emitLine("CONFIG_PRINTED");
            return SerialCommandResult::OK_EXECUTED;
        }
        if (strcmp(normalized, "COMMISSION") == 0 || strcmp(normalized, "COMMISSION STATUS") == 0) {
            emitLine("COMMISSION_PRINTED");
            return SerialCommandResult::OK_EXECUTED;
        }

        // Hardware driver topology configuration commands
        if (strcmp(normalized, "DRIVER VEH HIGH_SIDE_ACTIVE_HIGH") == 0 ||
            strcmp(normalized, "DRIVER VEH HIGH_SIDE_SOURCING_ACTIVE_HIGH") == 0) {
            return ctrl.configureVehicleDriver(nowMs, TrafficConfig::OutputDriverType::HIGH_SIDE_SOURCING_ACTIVE_HIGH)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }
        if (strcmp(normalized, "DRIVER VEH HIGH_SIDE_ACTIVE_LOW") == 0 ||
            strcmp(normalized, "DRIVER VEH HIGH_SIDE_SOURCING_ACTIVE_LOW") == 0) {
            return ctrl.configureVehicleDriver(nowMs, TrafficConfig::OutputDriverType::HIGH_SIDE_SOURCING_ACTIVE_LOW)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }
        if (strcmp(normalized, "DRIVER VEH DIRECT_ACTIVE_HIGH") == 0 ||
            strcmp(normalized, "DRIVER VEH DIRECT_LOGIC_ACTIVE_HIGH") == 0) {
            return ctrl.configureVehicleDriver(nowMs, TrafficConfig::OutputDriverType::DIRECT_ACTIVE_HIGH)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }
        if (strcmp(normalized, "DRIVER VEH LOW_SIDE_ULN2003A") == 0) {
            return ctrl.configureVehicleDriver(nowMs, TrafficConfig::OutputDriverType::LOW_SIDE_ULN2003A_ACTIVE_HIGH)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }
        if (strcmp(normalized, "DRIVER VEH DISABLED") == 0) {
            return ctrl.configureVehicleDriver(nowMs, TrafficConfig::OutputDriverType::UNVERIFIED_SAFE_DISABLED)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }

        if (strcmp(normalized, "DRIVER PED LOW_SIDE_ULN2003A") == 0 ||
            strcmp(normalized, "DRIVER PED LOW_SIDE_ULN2003A_ACTIVE_HIGH") == 0) {
            return ctrl.configurePedestrianDriver(nowMs, TrafficConfig::OutputDriverType::LOW_SIDE_ULN2003A_ACTIVE_HIGH)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }
        if (strcmp(normalized, "DRIVER PED HIGH_SIDE_ACTIVE_HIGH") == 0) {
            return ctrl.configurePedestrianDriver(nowMs, TrafficConfig::OutputDriverType::HIGH_SIDE_SOURCING_ACTIVE_HIGH)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }
        if (strcmp(normalized, "DRIVER PED HIGH_SIDE_ACTIVE_LOW") == 0) {
            return ctrl.configurePedestrianDriver(nowMs, TrafficConfig::OutputDriverType::HIGH_SIDE_SOURCING_ACTIVE_LOW)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }
        if (strcmp(normalized, "DRIVER PED DIRECT_ACTIVE_HIGH") == 0 ||
            strcmp(normalized, "DRIVER PED DIRECT_LOGIC_ACTIVE_HIGH") == 0) {
            return ctrl.configurePedestrianDriver(nowMs, TrafficConfig::OutputDriverType::DIRECT_ACTIVE_HIGH)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }
        if (strcmp(normalized, "DRIVER PED DISABLED") == 0) {
            return ctrl.configurePedestrianDriver(nowMs, TrafficConfig::OutputDriverType::UNVERIFIED_SAFE_DISABLED)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }

        // Runtime hardware commissioning commands
        if (strcmp(normalized, "COMMISSION VEH ON") == 0) {
            return ctrl.commissionVehicleDriver(nowMs, true)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }
        if (strcmp(normalized, "COMMISSION VEH OFF") == 0) {
            return ctrl.commissionVehicleDriver(nowMs, false)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }
        if (strcmp(normalized, "COMMISSION PED ON") == 0) {
            return ctrl.commissionPedestrianDriver(nowMs, true)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }
        if (strcmp(normalized, "COMMISSION PED OFF") == 0) {
            return ctrl.commissionPedestrianDriver(nowMs, false)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }
        if (strcmp(normalized, "COMMISSION IR ALL ON") == 0) {
            return ctrl.commissionAllIrSensors(nowMs, true)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }
        if (strcmp(normalized, "COMMISSION IR ALL OFF") == 0) {
            return ctrl.commissionAllIrSensors(nowMs, false)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }
        if (strncmp(normalized, "COMMISSION IR ", 14) == 0) {
            unsigned int chNum = 0;
            char stateStr[8] = {0};
            if (sscanf(normalized + 14, "%u %7s", &chNum, stateStr) == 2 &&
                chNum >= 1 && chNum <= TrafficConfig::IR_SENSOR_COUNT) {
                if (strcmp(stateStr, "ON") == 0) {
                    return ctrl.commissionIrSensorChannel(nowMs, static_cast<uint8_t>(chNum - 1), true)
                               ? SerialCommandResult::OK_EXECUTED
                               : SerialCommandResult::REJECTED_BY_STATE;
                }
                if (strcmp(stateStr, "OFF") == 0) {
                    return ctrl.commissionIrSensorChannel(nowMs, static_cast<uint8_t>(chNum - 1), false)
                               ? SerialCommandResult::OK_EXECUTED
                               : SerialCommandResult::REJECTED_BY_STATE;
                }
            }
            emitLine("COMMAND_REJECTED_UNKNOWN");
            return SerialCommandResult::UNKNOWN_INVALID_CMD;
        }
        if (strcmp(normalized, "COMMISSION TURN ON") == 0) {
            return ctrl.commissionProtectedTurn(nowMs, true)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }
        if (strcmp(normalized, "COMMISSION TURN OFF") == 0) {
            return ctrl.commissionProtectedTurn(nowMs, false)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }

        // Single-lamp commissioning test commands
        if (strcmp(normalized, "LAMP OFF") == 0 || strcmp(normalized, "LAMP TEST OFF") == 0) {
            ctrl.cancelLampDiagnostics(nowMs);
            ctrl.stopLampTest(nowMs);
            return SerialCommandResult::OK_EXECUTED;
        }
        if (strncmp(normalized, "LAMP TEST ", 10) == 0) {
            const int8_t chIdx = SignalOutputDriver::parseChannelName(normalized + 10);
            if (chIdx >= 0) {
                return ctrl.startLampTest(nowMs, chIdx)
                           ? SerialCommandResult::OK_EXECUTED
                           : SerialCommandResult::REJECTED_BY_STATE;
            }
            emitLine("COMMAND_REJECTED_UNKNOWN");
            return SerialCommandResult::UNKNOWN_INVALID_CMD;
        }

        // Automated individual lamp diagnostics
        if (strcmp(normalized, "DIAG LAMPS") == 0 || strcmp(normalized, "DIAG LAMPS VEH") == 0) {
            return ctrl.startLampDiagnostics(nowMs, 1000, 1)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }
        if (strcmp(normalized, "DIAG LAMPS PED") == 0) {
            return ctrl.startLampDiagnostics(nowMs, 1000, 2)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }
        if (strcmp(normalized, "DIAG LAMPS ALL") == 0 || strcmp(normalized, "DIAG 13") == 0) {
            return ctrl.startLampDiagnostics(nowMs, 1000, 0)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }
        if (strncmp(normalized, "DIAG LAMPS VEH ", 15) == 0) {
            unsigned long dur = 1000;
            if (sscanf(normalized + 15, "%lu", &dur) == 1) {
                return ctrl.startLampDiagnostics(nowMs, dur, 1)
                           ? SerialCommandResult::OK_EXECUTED
                           : SerialCommandResult::REJECTED_BY_STATE;
            }
            emitLine("COMMAND_REJECTED_UNKNOWN");
            return SerialCommandResult::UNKNOWN_INVALID_CMD;
        }
        if (strncmp(normalized, "DIAG LAMPS PED ", 15) == 0) {
            unsigned long dur = 1000;
            if (sscanf(normalized + 15, "%lu", &dur) == 1) {
                return ctrl.startLampDiagnostics(nowMs, dur, 2)
                           ? SerialCommandResult::OK_EXECUTED
                           : SerialCommandResult::REJECTED_BY_STATE;
            }
            emitLine("COMMAND_REJECTED_UNKNOWN");
            return SerialCommandResult::UNKNOWN_INVALID_CMD;
        }
        if (strncmp(normalized, "DIAG LAMPS ALL ", 15) == 0) {
            unsigned long dur = 1000;
            if (sscanf(normalized + 15, "%lu", &dur) == 1) {
                return ctrl.startLampDiagnostics(nowMs, dur, 0)
                           ? SerialCommandResult::OK_EXECUTED
                           : SerialCommandResult::REJECTED_BY_STATE;
            }
            emitLine("COMMAND_REJECTED_UNKNOWN");
            return SerialCommandResult::UNKNOWN_INVALID_CMD;
        }
        if (strncmp(normalized, "DIAG LAMPS ", 11) == 0) {
            unsigned long dur = 1000;
            if (sscanf(normalized + 11, "%lu", &dur) == 1) {
                return ctrl.startLampDiagnostics(nowMs, dur, 1)
                           ? SerialCommandResult::OK_EXECUTED
                           : SerialCommandResult::REJECTED_BY_STATE;
            }
            emitLine("COMMAND_REJECTED_UNKNOWN");
            return SerialCommandResult::UNKNOWN_INVALID_CMD;
        }
        if (strcmp(normalized, "DIAG STOP") == 0 || strcmp(normalized, "DIAG CANCEL") == 0 ||
            strcmp(normalized, "TEST STOP") == 0 || strcmp(normalized, "TEST ALL_GREEN STOP") == 0 ||
            strcmp(normalized, "TEST ALL_GREEN CANCEL") == 0) {
            ctrl.cancelLampDiagnostics(nowMs);
            ctrl.cancelAllGreenDiagnostics(nowMs, "SERIAL_OPERATOR_ABORT");
            return SerialCommandResult::OK_EXECUTED;
        }
        if (strcmp(normalized, "DIAG") == 0 || strcmp(normalized, "DIAG REPORT") == 0 ||
            strcmp(normalized, "DIAG STATUS") == 0 || strcmp(normalized, "TEST ALL_GREEN REPORT") == 0) {
            emitLine("DIAG_REPORT_PRINTED");
            return SerialCommandResult::OK_EXECUTED;
        }

        // Explicit Isolated Bench Diagnostic Safety Authorization
        if (strcmp(normalized, "BENCH UNLOCK 0x42454E43") == 0 || strcmp(normalized, "BENCH UNLOCK BENCH") == 0) {
            if (!TrafficConfig::PERMIT_ALL_GREEN_BENCH_TEST) {
                emitLine("COMMAND_REJECTED: Multi-green bench mode is disabled in commissioning firmware.");
                return SerialCommandResult::REJECTED_BY_STATE;
            }
            ctrl.unlockBenchDiagnostics(true);
            emitLine("EVENT: BENCH_DIAGNOSTICS_UNLOCKED | Multi-green bench mode permitted for isolated testing");
            return SerialCommandResult::OK_EXECUTED;
        }
        if (strcmp(normalized, "BENCH LOCK") == 0) {
            ctrl.unlockBenchDiagnostics(false);
            ctrl.cancelAllGreenDiagnostics(nowMs, "BENCH_LOCKED");
            emitLine("EVENT: BENCH_DIAGNOSTICS_LOCKED | Simultaneous greens strictly prohibited");
            return SerialCommandResult::OK_EXECUTED;
        }

        // Controlled 10-Second ALL-GREEN Diagnostic Test (Hardware Bench Diagnostic Only)
        if (strcmp(normalized, "TEST ALL_GREEN") == 0 || strcmp(normalized, "DIAG ALL_GREEN") == 0) {
            if (!TrafficConfig::PERMIT_ALL_GREEN_BENCH_TEST) {
                emitLine("COMMAND_REJECTED: ALL-GREEN test disabled in commissioning firmware. Simultaneous vehicle greens strictly prohibited.");
                return SerialCommandResult::REJECTED_BY_STATE;
            }
            return ctrl.startAllGreenDiagnostics(nowMs, 10000)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }
        if (strncmp(normalized, "TEST ALL_GREEN ", 15) == 0) {
            if (!TrafficConfig::PERMIT_ALL_GREEN_BENCH_TEST) {
                emitLine("COMMAND_REJECTED: ALL-GREEN test disabled in commissioning firmware. Simultaneous vehicle greens strictly prohibited.");
                return SerialCommandResult::REJECTED_BY_STATE;
            }
            unsigned long durSec = 10;
            if (sscanf(normalized + 15, "%lu", &durSec) == 1 && durSec >= 1 && durSec <= 30) {
                return ctrl.startAllGreenDiagnostics(nowMs, durSec * 1000UL)
                           ? SerialCommandResult::OK_EXECUTED
                           : SerialCommandResult::REJECTED_BY_STATE;
            }
            emitLine("COMMAND_REJECTED_UNKNOWN");
            return SerialCommandResult::UNKNOWN_INVALID_CMD;
        }

        // Mode commands
        if (strcmp(normalized, "MODE AUTO") == 0) {
            return ctrl.setControlMode(nowMs, ControlMode::AUTO)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }
        if (strcmp(normalized, "MODE FIXED") == 0) {
            return ctrl.setControlMode(nowMs, ControlMode::FIXED)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }
        if (strcmp(normalized, "MODE ACTUATED") == 0) {
            return ctrl.setControlMode(nowMs, ControlMode::ACTUATED)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }
        if (strcmp(normalized, "MODE ADAPTIVE") == 0) {
            return ctrl.setControlMode(nowMs, ControlMode::ADAPTIVE)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }
        if (strcmp(normalized, "MODE MANUAL") == 0) {
            return ctrl.setControlMode(nowMs, ControlMode::MANUAL)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }

        // Simulation mode commands
        if (strcmp(normalized, "SIM ON") == 0) {
            return ctrl.setSimulationMode(nowMs, true)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }
        if (strcmp(normalized, "SIM OFF") == 0) {
            return ctrl.setSimulationMode(nowMs, false)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }

        // Pedestrian request commands
        if (strcmp(normalized, "PED 1") == 0 || strcmp(normalized, "PED") == 0 || strcmp(normalized, "PEDESTRIAN") == 0) {
            if (ctrl.getMode() != OperatingMode::SERIAL_SIMULATION) {
                ctrl.setSimulationMode(nowMs, true);
            }
            return ctrl.triggerSimPedestrianChannel(nowMs, 0)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }
        if (strcmp(normalized, "PED 2") == 0) {
            if (ctrl.getMode() != OperatingMode::SERIAL_SIMULATION) {
                ctrl.setSimulationMode(nowMs, true);
            }
            return ctrl.triggerSimPedestrianChannel(nowMs, 1)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }

        // Vehicle demand commands (CAR ON / CAR OFF / VEHICLE ON / VEHICLE OFF / CAR MAIN|SIDE|TURN ON|OFF)
        if (strcmp(normalized, "CAR ON") == 0 || strcmp(normalized, "VEHICLE ON") == 0 || strcmp(normalized, "CAR SIDE ON") == 0) {
            if (ctrl.getMode() != OperatingMode::SERIAL_SIMULATION) {
                ctrl.setSimulationMode(nowMs, true);
            }
            return ctrl.setSimVehicleDemand(nowMs, true)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }
        if (strcmp(normalized, "CAR OFF") == 0 || strcmp(normalized, "VEHICLE OFF") == 0 || strcmp(normalized, "CAR SIDE OFF") == 0) {
            if (ctrl.getMode() != OperatingMode::SERIAL_SIMULATION) {
                ctrl.setSimulationMode(nowMs, true);
            }
            return ctrl.setSimVehicleDemand(nowMs, false)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }
        if (strcmp(normalized, "CAR MAIN ON") == 0) {
            if (ctrl.getMode() != OperatingMode::SERIAL_SIMULATION) ctrl.setSimulationMode(nowMs, true);
            return ctrl.setSimMovementDemand(nowMs, TrafficConfig::MovementId::MAIN_ROAD, true)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }
        if (strcmp(normalized, "CAR MAIN OFF") == 0) {
            if (ctrl.getMode() != OperatingMode::SERIAL_SIMULATION) ctrl.setSimulationMode(nowMs, true);
            return ctrl.setSimMovementDemand(nowMs, TrafficConfig::MovementId::MAIN_ROAD, false)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }
        if (strcmp(normalized, "CAR TURN ON") == 0) {
            if (ctrl.getMode() != OperatingMode::SERIAL_SIMULATION) ctrl.setSimulationMode(nowMs, true);
            return ctrl.setSimMovementDemand(nowMs, TrafficConfig::MovementId::PROTECTED_TURN, true)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }
        if (strcmp(normalized, "CAR TURN OFF") == 0) {
            if (ctrl.getMode() != OperatingMode::SERIAL_SIMULATION) ctrl.setSimulationMode(nowMs, true);
            return ctrl.setSimMovementDemand(nowMs, TrafficConfig::MovementId::PROTECTED_TURN, false)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }

        // Manual step command
        if (strcmp(normalized, "NEXT") == 0) {
            return ctrl.advanceManualStep(nowMs)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }

        // Fault injection & recovery commands
        if (strcmp(normalized, "FAULT") == 0) {
            ctrl.triggerFault(nowMs, "SERIAL_OPERATOR_FAULT", FaultCode::OPERATOR_INJECTED_FAULT);
            return SerialCommandResult::OK_EXECUTED;
        }
        if (strcmp(normalized, "CLEAR") == 0) {
            if (ctrl.getState() == TrafficState::FAULT_SAFE) {
                ctrl.clearFaultAndRecover(nowMs, "SERIAL_CLEAR_FAULT_RECOVERY");
                return SerialCommandResult::OK_EXECUTED;
            }
            if (ctrl.getMode() == OperatingMode::SERIAL_SIMULATION) {
                ctrl.clearSimulatedDemand(nowMs);
                ctrl.clearNonFatalFaults(nowMs);
                return SerialCommandResult::OK_EXECUTED;
            }
            if (ctrl.clearNonFatalFaults(nowMs)) {
                return SerialCommandResult::OK_EXECUTED;
            }
            return SerialCommandResult::REJECTED_BY_STATE;
        }
        if (strcmp(normalized, "RESET") == 0) {
            ctrl.resetLogical(nowMs, "SERIAL_RESET_COMMAND");
            return SerialCommandResult::OK_EXECUTED;
        }

        // Persistent NVS configuration commands
        if (strcmp(normalized, "SAVE") == 0) {
            return ctrl.saveConfigToNvs(nowMs)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }
        if (strcmp(normalized, "LOAD") == 0) {
            return ctrl.loadConfigFromNvs(nowMs)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }
        if (strcmp(normalized, "DEFAULTS") == 0) {
            return ctrl.restoreDefaultConfig(nowMs)
                       ? SerialCommandResult::OK_EXECUTED
                       : SerialCommandResult::REJECTED_BY_STATE;
        }
        if (strcmp(normalized, "REBOOT") == 0) {
            ctrl.resetLogical(nowMs, "SERIAL_REBOOT_SAFE_PREPARATION");
            rebootRequested_ = true;
            return SerialCommandResult::OK_EXECUTED;
        }

        emitLine("COMMAND_REJECTED_UNKNOWN");
        return SerialCommandResult::UNKNOWN_INVALID_CMD;
    }

    static DashboardStatusSnapshot captureStatusSnapshot(uint32_t nowMs, const TrafficController& ctrl) {
        DashboardStatusSnapshot snap{};
        snap.timestampMs               = nowMs;
        snap.rightHandTrafficRule      = ctrl.getConflictMatrixConfig().rightHandTrafficRule;
        snap.operatingMode             = ctrl.getMode();
        snap.controlMode               = ctrl.getControlMode();
        snap.currentFsmState           = ctrl.getState();
        snap.stateElapsedMs            = ctrl.getStateElapsed(nowMs);
        snap.sideGapActive             = ctrl.isSideGapTimerActive();
        snap.sideGapElapsedMs          = ctrl.getSideGapElapsed(nowMs);
        snap.nextTargetPhase           = ctrl.getNextTarget();
        snap.protectedTurnCommissioned = ctrl.isProtectedTurnCommissioned();
        snap.mainDemand                = ctrl.hasMainVehicleDemand();
        snap.sideDemand                = ctrl.hasSideVehicleDemand();
        snap.turnDemand                = ctrl.hasTurnVehicleDemand();
        snap.ped1Demand                = ctrl.hasPedestrianChannelDemand(0);
        snap.ped2Demand                = ctrl.hasPedestrianChannelDemand(1);
        snap.pedClearanceActive        = ctrl.isPedestrianClearanceActive();
        snap.validatedIrMask           = ctrl.getSensorConfig().sensorValidatedMask;
        snap.physicalIrValid           = (snap.validatedIrMask != 0);
        snap.outputs                   = ctrl.getOutputs();
        const ElectricalPinStates& elSt = ctrl.getOutputDriver().getLastElectricalStates();
        snap.electricalOutputs.mainGreen    = elSt.pinHigh[0];
        snap.electricalOutputs.mainYellow   = elSt.pinHigh[1];
        snap.electricalOutputs.mainRed      = elSt.pinHigh[2];
        snap.electricalOutputs.sideGreen    = elSt.pinHigh[3];
        snap.electricalOutputs.sideYellow   = elSt.pinHigh[4];
        snap.electricalOutputs.sideRed      = elSt.pinHigh[5];
        snap.electricalOutputs.turnGreen    = elSt.pinHigh[6];
        snap.electricalOutputs.turnYellow   = elSt.pinHigh[7];
        snap.electricalOutputs.turnRed      = elSt.pinHigh[8];
        snap.electricalOutputs.pedWalk      = elSt.pinHigh[9];
        snap.electricalOutputs.pedDontWalk  = elSt.pinHigh[10];
        snap.electricalOutputs.ped2Walk     = elSt.pinHigh[11];
        snap.electricalOutputs.ped2DontWalk = elSt.pinHigh[12];
        snap.activeFaultMask           = ctrl.getFaultManager().getActiveFaultMask();
        snap.valid                     = snap.rightHandTrafficRule;
        return snap;
    }

    static void formatLogicalOutputsLine(const SignalOutputs& out, char* buf, size_t bufSize) {
        if (!buf || bufSize == 0) return;
        snprintf(buf, bufSize,
                 "MAIN[G=%d Y=%d R=%d] SIDE[G=%d Y=%d R=%d] TURN[G=%d Y=%d R=%d] PED1[W=%d DW=%d] PED2[W=%d DW=%d]",
                 out.mainGreen ? 1 : 0, out.mainYellow ? 1 : 0, out.mainRed ? 1 : 0,
                 out.sideGreen ? 1 : 0, out.sideYellow ? 1 : 0, out.sideRed ? 1 : 0,
                 out.turnGreen ? 1 : 0, out.turnYellow ? 1 : 0, out.turnRed ? 1 : 0,
                 out.pedWalk ? 1 : 0, out.pedDontWalk ? 1 : 0,
                 out.ped2Walk ? 1 : 0, out.ped2DontWalk ? 1 : 0);
    }

    static void formatElectricalPinStatesLine(const ElectricalPinStates& st, char* buf, size_t bufSize) {
        if (!buf || bufSize == 0) return;
        snprintf(buf, bufSize,
                 "MAIN[G=%d Y=%d R=%d] SIDE[G=%d Y=%d R=%d] TURN[G=%d Y=%d R=%d] PED1[W=%d DW=%d] PED2[W=%d DW=%d]",
                 st.pinHigh[0] ? 1 : 0, st.pinHigh[1] ? 1 : 0, st.pinHigh[2] ? 1 : 0,
                 st.pinHigh[3] ? 1 : 0, st.pinHigh[4] ? 1 : 0, st.pinHigh[5] ? 1 : 0,
                 st.pinHigh[6] ? 1 : 0, st.pinHigh[7] ? 1 : 0, st.pinHigh[8] ? 1 : 0,
                 st.pinHigh[9] ? 1 : 0, st.pinHigh[10] ? 1 : 0,
                 st.pinHigh[11] ? 1 : 0, st.pinHigh[12] ? 1 : 0);
    }

    static bool parseLogicalOutputsLine(const char* line, SignalOutputs& out) {
        if (!line) return false;
        int mg = 0, my = 0, mr = 0;
        int sg = 0, sy = 0, sr = 0;
        int tg = 0, ty = 0, tr = 0;
        int p1w = 0, p1dw = 0;
        int p2w = 0, p2dw = 0;

        const char* start = strstr(line, "MAIN[");
        if (!start) start = line;

        const int matched = sscanf(
            start,
            "MAIN[G=%d Y=%d R=%d] SIDE[G=%d Y=%d R=%d] TURN[G=%d Y=%d R=%d] PED1[W=%d DW=%d] PED2[W=%d DW=%d]",
            &mg, &my, &mr, &sg, &sy, &sr, &tg, &ty, &tr, &p1w, &p1dw, &p2w, &p2dw);

        if (matched == 13) {
            out.mainGreen    = (mg != 0);
            out.mainYellow   = (my != 0);
            out.mainRed      = (mr != 0);
            out.sideGreen    = (sg != 0);
            out.sideYellow   = (sy != 0);
            out.sideRed      = (sr != 0);
            out.turnGreen    = (tg != 0);
            out.turnYellow   = (ty != 0);
            out.turnRed      = (tr != 0);
            out.pedWalk      = (p1w != 0);
            out.pedDontWalk  = (p1dw != 0);
            out.ped2Walk     = (p2w != 0);
            out.ped2DontWalk = (p2dw != 0);
            return true;
        }
        return false;
    }

private:
    void emitLine(const char* msg) {
        if (printCb_) {
            printCb_(msg);
        }
    }

    OutputPrintCallback printCb_;
    bool                rebootRequested_;
};

#endif // SERIAL_COMMAND_HANDLER_H

