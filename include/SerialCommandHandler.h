#ifndef SERIAL_COMMAND_HANDLER_H
#define SERIAL_COMMAND_HANDLER_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <ctype.h>
#include "TrafficController.h"

enum class SerialCommandResult : uint8_t {
    OK_EXECUTED         = 0,
    EMPTY_IGNORED       = 1,
    REJECTED_BY_STATE   = 2,
    UNKNOWN_INVALID_CMD = 3
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

        char buf[96];
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
        char normalized[96];
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
        if (strcmp(normalized, "CONFIG") == 0) {
            emitLine("CONFIG_PRINTED");
            return SerialCommandResult::OK_EXECUTED;
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

