#ifndef FAULT_MANAGER_H
#define FAULT_MANAGER_H

#include <stdint.h>
#include "TrafficConfig.h"

enum class FaultCode : uint16_t {
    NONE                          = 0x0000,
    INVALID_SENSOR_STATE          = 0x0001,
    SENSOR_STUCK_ACTIVE           = 0x0002,
    SENSOR_DISCONNECTED_CHATTER   = 0x0004,
    PED_BUTTON_STUCK_ACTIVE       = 0x0008,
    INVALID_CONFIGURATION         = 0x0010,
    ILLEGAL_SIGNAL_COMBINATION    = 0x0020,
    CONTROLLER_TIMING_FAULT       = 0x0040,
    UNEXPECTED_STATE_TRANSITION   = 0x0080,
    UNCOMMISSIONED_TURN_CONFLICT  = 0x0100,
    OPERATOR_INJECTED_FAULT       = 0x0200
};

class FaultManager {
public:
    FaultManager() { reset(0); }

    void reset(uint32_t nowMs) {
        activeFaultMask_   = 0;
        criticalFaultLatched_ = false;
        lastFaultCode_     = FaultCode::NONE;
        lastFaultReason_   = "NONE";
        lastFaultTimeMs_   = nowMs;
        faultCount_        = 0;
        sensorFaultMask_   = 0;
        pedButtonFaultMask_ = 0;
    }

    // Record a critical safety fault that forces the controller into FAULT_SAFE
    void recordCriticalFault(uint32_t nowMs, FaultCode code, const char* reason) {
        activeFaultMask_      |= static_cast<uint16_t>(code);
        criticalFaultLatched_  = true;
        lastFaultCode_         = code;
        lastFaultReason_       = (reason && reason[0] != '\0') ? reason : faultCodeToString(code);
        lastFaultTimeMs_       = nowMs;
        if (faultCount_ < 0xFFFF) {
            faultCount_++;
        }
    }

    // Record a non-fatal degraded hardware fault (e.g., single stuck/noisy sensor or button)
    // that isolates the faulty input without causing an unsafe green decision.
    void recordNonFatalFault(uint32_t nowMs, FaultCode code, const char* reason) {
        activeFaultMask_ |= static_cast<uint16_t>(code);
        lastFaultCode_    = code;
        lastFaultReason_  = (reason && reason[0] != '\0') ? reason : faultCodeToString(code);
        lastFaultTimeMs_  = nowMs;
        if (faultCount_ < 0xFFFF) {
            faultCount_++;
        }
    }

    void setSensorFaultBit(uint8_t sensorIdx, bool faulted) {
        if (sensorIdx >= TrafficConfig::IR_SENSOR_COUNT) return;
        if (faulted) {
            sensorFaultMask_ |= static_cast<uint8_t>(1U << sensorIdx);
        } else {
            sensorFaultMask_ &= static_cast<uint8_t>(~(1U << sensorIdx));
            if (sensorFaultMask_ == 0) {
                activeFaultMask_ &= static_cast<uint16_t>(~(
                    static_cast<uint16_t>(FaultCode::SENSOR_STUCK_ACTIVE) |
                    static_cast<uint16_t>(FaultCode::SENSOR_DISCONNECTED_CHATTER)));
                refreshIdleFaultReason();
            }
        }
    }

    void setPedButtonFaultBit(uint8_t btnIdx, bool faulted) {
        if (btnIdx >= TrafficConfig::PED_BUTTON_COUNT) return;
        if (faulted) {
            pedButtonFaultMask_ |= static_cast<uint8_t>(1U << btnIdx);
        } else {
            pedButtonFaultMask_ &= static_cast<uint8_t>(~(1U << btnIdx));
            if (pedButtonFaultMask_ == 0) {
                activeFaultMask_ &= static_cast<uint16_t>(~static_cast<uint16_t>(FaultCode::PED_BUTTON_STUCK_ACTIVE));
                refreshIdleFaultReason();
            }
        }
    }

    void clearFaultBit(FaultCode code) {
        activeFaultMask_ &= static_cast<uint16_t>(~static_cast<uint16_t>(code));
        refreshIdleFaultReason();
    }

    void clearNonFatalFaults(uint32_t nowMs) {
        if (criticalFaultLatched_) {
            return;
        }
        activeFaultMask_    = 0;
        sensorFaultMask_    = 0;
        pedButtonFaultMask_ = 0;
        lastFaultCode_      = FaultCode::NONE;
        lastFaultReason_    = "NONE";
        lastFaultTimeMs_    = nowMs;
    }

    // Explicit recovery procedure required to exit critical fault state
    void clearCriticalFault(uint32_t nowMs) {
        criticalFaultLatched_ = false;
        activeFaultMask_      = 0;
        sensorFaultMask_      = 0;
        pedButtonFaultMask_   = 0;
        lastFaultCode_        = FaultCode::NONE;
        lastFaultReason_      = "NONE";
        lastFaultTimeMs_      = nowMs;
    }

    bool isCriticalFaultLatched() const { return criticalFaultLatched_; }
    uint16_t getActiveFaultMask() const { return activeFaultMask_; }
    FaultCode getLastFaultCode() const { return lastFaultCode_; }
    const char* getLastFaultReason() const { return lastFaultReason_; }
    uint32_t getLastFaultTimeMs() const { return lastFaultTimeMs_; }
    uint16_t getFaultCount() const { return faultCount_; }
    uint8_t getSensorFaultMask() const { return sensorFaultMask_; }
    uint8_t getPedButtonFaultMask() const { return pedButtonFaultMask_; }

    static const char* faultCodeToString(FaultCode code) {
        switch (code) {
            case FaultCode::NONE:                        return "NONE";
            case FaultCode::INVALID_SENSOR_STATE:        return "INVALID_SENSOR_STATE";
            case FaultCode::SENSOR_STUCK_ACTIVE:         return "SENSOR_STUCK_ACTIVE";
            case FaultCode::SENSOR_DISCONNECTED_CHATTER: return "SENSOR_DISCONNECTED_CHATTER";
            case FaultCode::PED_BUTTON_STUCK_ACTIVE:     return "PED_BUTTON_STUCK_ACTIVE";
            case FaultCode::INVALID_CONFIGURATION:       return "INVALID_CONFIGURATION";
            case FaultCode::ILLEGAL_SIGNAL_COMBINATION:  return "ILLEGAL_SIGNAL_COMBINATION";
            case FaultCode::CONTROLLER_TIMING_FAULT:     return "CONTROLLER_TIMING_FAULT";
            case FaultCode::UNEXPECTED_STATE_TRANSITION: return "UNEXPECTED_STATE_TRANSITION";
            case FaultCode::UNCOMMISSIONED_TURN_CONFLICT:return "UNCOMMISSIONED_TURN_CONFLICT";
            case FaultCode::OPERATOR_INJECTED_FAULT:     return "OPERATOR_INJECTED_FAULT";
            default:                                     return "UNKNOWN_FAULT";
        }
    }

private:
    void refreshIdleFaultReason() {
        if (activeFaultMask_ == 0 && !criticalFaultLatched_) {
            lastFaultCode_   = FaultCode::NONE;
            lastFaultReason_ = "NONE";
        }
    }

    uint16_t    activeFaultMask_;
    bool        criticalFaultLatched_;
    FaultCode   lastFaultCode_;
    const char* lastFaultReason_;
    uint32_t    lastFaultTimeMs_;
    uint16_t    faultCount_;
    uint8_t     sensorFaultMask_;
    uint8_t     pedButtonFaultMask_;
};

#endif // FAULT_MANAGER_H

