#ifndef SIGNAL_OUTPUT_DRIVER_H
#define SIGNAL_OUTPUT_DRIVER_H

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "TrafficConfig.h"

#ifdef ARDUINO
#include <Arduino.h>
#endif

// ============================================================================
// SIGNAL OUTPUT DRIVER (Central Hardware Abstraction Layer)
// ============================================================================
// Separates logical traffic signal state from electrical GPIO and ULN2003A
// driver behaviour across all 13 signal channels:
//   - 9 Vehicle RYG channels:
//       Main Road     : Red = GPIO 27, Yellow = GPIO 26, Green = GPIO 25
//       Side Road     : Red = GPIO 14, Yellow = GPIO 12, Green = GPIO 13
//       Turn Movement : Red = GPIO 22, Yellow = GPIO 23, Green = GPIO 21
//   - 4 Pedestrian channels:
//       Pedestrian 1  : Walk = GPIO 19, Don't Walk = GPIO 18
//       Pedestrian 2  : Walk = GPIO 16, Don't Walk = GPIO 17
//
// Strictly distinguishes four distinct layers per channel:
//   1. Logical signal state   (logicalState[i])     : FSM intent (true = ON, false = OFF)
//   2. ESP32 GPIO state       (pinHigh[i])          : MCU pin voltage (true = 3.3V HIGH, false = 0V LOW)
//   3. ULN2003A sink state    (ulnSinkActive[i])    : Darlington sink (true = sinking to GND, false = open-collector)
//   4. Physical lamp state    (channelEnergized[i]) : Actual lamp illumination (true = lit, false = dark)
//
// Never assumes "MCU GPIO HIGH == physical lamp ON" or "MCU GPIO LOW == lamp OFF".

struct LogicalSignalOutputs {
    // Core v1.1.0 fields (1..8) — preserved in exact order for aggregate compatibility
    bool mainGreen;     // Main Road Green      (GPIO 25)
    bool mainYellow;    // Main Road Yellow     (GPIO 26)
    bool mainRed;       // Main Road Red        (GPIO 27)
    bool sideGreen;     // Side Road Green      (GPIO 13)
    bool sideYellow;    // Side Road Yellow     (GPIO 12)
    bool sideRed;       // Side Road Red        (GPIO 14)
    bool pedWalk;       // Pedestrian 1 Walk    (GPIO 19)
    bool pedDontWalk;   // Pedestrian 1 Don't W (GPIO 18)

    // Extended v2.0.0 fields (9..13)
    bool turnGreen;     // Turn Movement Green  (GPIO 21)
    bool turnYellow;    // Turn Movement Yellow (GPIO 23)
    bool turnRed;       // Turn Movement Red    (GPIO 22)
    bool ped2Walk;      // Pedestrian 2 Walk    (GPIO 16)
    bool ped2DontWalk;  // Pedestrian 2 Don't W (GPIO 17)
};

struct ElectricalPinStates {
    bool    logicalState[TrafficConfig::TOTAL_SIGNAL_OUTPUTS];     // Layer 1: FSM logical signal state
    bool    pinHigh[TrafficConfig::TOTAL_SIGNAL_OUTPUTS];          // Layer 2: ESP32 GPIO output level (HIGH=true, LOW=false)
    bool    ulnSinkActive[TrafficConfig::TOTAL_SIGNAL_OUTPUTS];    // Layer 3: ULN2003A Darlington sink active (sinking to GND)
    bool    channelEnergized[TrafficConfig::TOTAL_SIGNAL_OUTPUTS]; // Layer 4: Physical traffic-light lamp illuminated
    uint8_t gpioPin[TrafficConfig::TOTAL_SIGNAL_OUTPUTS];          // Authoritative physical GPIO number for each channel

    int8_t findChannelByGpio(uint8_t targetGpio) const {
        for (uint8_t i = 0; i < TrafficConfig::TOTAL_SIGNAL_OUTPUTS; ++i) {
            if (gpioPin[i] == targetGpio) {
                return static_cast<int8_t>(i);
            }
        }
        return -1;
    }

    bool isLogicalSignalOnForGpio(uint8_t targetGpio) const {
        const int8_t idx = findChannelByGpio(targetGpio);
        return (idx >= 0) ? logicalState[idx] : false;
    }

    bool isGpioDrivenHigh(uint8_t targetGpio) const {
        const int8_t idx = findChannelByGpio(targetGpio);
        return (idx >= 0) ? pinHigh[idx] : false;
    }

    bool isUlnSinkActiveForGpio(uint8_t targetGpio) const {
        const int8_t idx = findChannelByGpio(targetGpio);
        return (idx >= 0) ? ulnSinkActive[idx] : false;
    }

    bool isLampEnergizedForGpio(uint8_t targetGpio) const {
        const int8_t idx = findChannelByGpio(targetGpio);
        return (idx >= 0) ? channelEnergized[idx] : false;
    }
};

class SignalOutputDriver {
public:
    SignalOutputDriver()
        : vehicleDriverType_(TrafficConfig::OutputDriverType::DIRECT_ACTIVE_HIGH),
          pedDriverType_(TrafficConfig::OutputDriverType::LOW_SIDE_ULN2003A_ACTIVE_HIGH),
          vehicleDriverVerified_(TrafficConfig::VEHICLE_DRIVER_VERIFIED),
          pedDriverVerified_(TrafficConfig::PED_ULN2003A_DRIVER_VERIFIED),
          sideYellowPin_(TrafficConfig::PIN_SIDE_YELLOW),
          lampTestChannel_(-1),
          pins_{},
          lastElectrical_{} {
        clearElectricalStates();
    }

    void setVehicleDriverConfig(TrafficConfig::OutputDriverType type, bool verified) {
        vehicleDriverType_     = type;
        vehicleDriverVerified_ = verified;
        if (!verified || type == TrafficConfig::OutputDriverType::UNVERIFIED_SAFE_DISABLED) {
            if (lampTestChannel_ >= 0 && lampTestChannel_ < 9) {
                lampTestChannel_ = -1;
            }
        }
    }

    void setPedestrianDriverConfig(TrafficConfig::OutputDriverType type, bool verified) {
        pedDriverType_     = type;
        pedDriverVerified_ = verified;
        if (!verified || type == TrafficConfig::OutputDriverType::UNVERIFIED_SAFE_DISABLED) {
            if (lampTestChannel_ >= 9 && lampTestChannel_ < static_cast<int8_t>(TrafficConfig::TOTAL_SIGNAL_OUTPUTS)) {
                lampTestChannel_ = -1;
            }
        }
    }

    bool setLampTestChannel(int8_t channelIdx) {
        if (channelIdx < 0) {
            lampTestChannel_ = -1;
            return true;
        }
        if (channelIdx >= static_cast<int8_t>(TrafficConfig::TOTAL_SIGNAL_OUTPUTS)) {
            return false;
        }
        if (channelIdx < 9) {
            if (!vehicleDriverVerified_ ||
                vehicleDriverType_ == TrafficConfig::OutputDriverType::UNVERIFIED_SAFE_DISABLED) {
                return false;
            }
        } else {
            if (!pedDriverVerified_ ||
                pedDriverType_ == TrafficConfig::OutputDriverType::UNVERIFIED_SAFE_DISABLED) {
                return false;
            }
        }
        lampTestChannel_ = channelIdx;
        return true;
    }

    void clearLampTest() { lampTestChannel_ = -1; }
    int8_t getLampTestChannel() const { return lampTestChannel_; }
    bool isLampTestActive() const { return lampTestChannel_ >= 0; }

    static LogicalSignalOutputs buildSingleLampLogicalOutputs(int8_t channelIdx) {
        LogicalSignalOutputs out = {
            false, false, false, false, false, false, false, false,
            false, false, false, false, false
        };
        switch (channelIdx) {
            case 0:  out.mainGreen    = true; break;
            case 1:  out.mainYellow   = true; break;
            case 2:  out.mainRed      = true; break;
            case 3:  out.sideGreen    = true; break;
            case 4:  out.sideYellow   = true; break;
            case 5:  out.sideRed      = true; break;
            case 6:  out.turnGreen    = true; break;
            case 7:  out.turnYellow   = true; break;
            case 8:  out.turnRed      = true; break;
            case 9:  out.pedWalk      = true; break;
            case 10: out.pedDontWalk  = true; break;
            case 11: out.ped2Walk     = true; break;
            case 12: out.ped2DontWalk = true; break;
            default: break;
        }
        return out;
    }

    static const char* channelIndexToName(int8_t channelIdx) {
        switch (channelIdx) {
            case 0:  return "MAIN_G";
            case 1:  return "MAIN_Y";
            case 2:  return "MAIN_R";
            case 3:  return "SIDE_G";
            case 4:  return "SIDE_Y";
            case 5:  return "SIDE_R";
            case 6:  return "TURN_G";
            case 7:  return "TURN_Y";
            case 8:  return "TURN_R";
            case 9:  return "PED1_W";
            case 10: return "PED1_DW";
            case 11: return "PED2_W";
            case 12: return "PED2_DW";
            default: return "OFF";
        }
    }

    static int8_t parseChannelName(const char* name) {
        if (!name) return -2;
        if (strcmp(name, "OFF") == 0 || strcmp(name, "NONE") == 0) return -1;
        if (strcmp(name, "MAIN_G") == 0 || strcmp(name, "MAIN_GREEN") == 0) return 0;
        if (strcmp(name, "MAIN_Y") == 0 || strcmp(name, "MAIN_YELLOW") == 0) return 1;
        if (strcmp(name, "MAIN_R") == 0 || strcmp(name, "MAIN_RED") == 0) return 2;
        if (strcmp(name, "SIDE_G") == 0 || strcmp(name, "SIDE_GREEN") == 0) return 3;
        if (strcmp(name, "SIDE_Y") == 0 || strcmp(name, "SIDE_YELLOW") == 0) return 4;
        if (strcmp(name, "SIDE_R") == 0 || strcmp(name, "SIDE_RED") == 0) return 5;
        if (strcmp(name, "TURN_G") == 0 || strcmp(name, "TURN_GREEN") == 0) return 6;
        if (strcmp(name, "TURN_Y") == 0 || strcmp(name, "TURN_YELLOW") == 0) return 7;
        if (strcmp(name, "TURN_R") == 0 || strcmp(name, "TURN_RED") == 0) return 8;
        if (strcmp(name, "PED1_W") == 0 || strcmp(name, "PED1_WALK") == 0) return 9;
        if (strcmp(name, "PED1_DW") == 0 || strcmp(name, "PED1_DONT_WALK") == 0) return 10;
        if (strcmp(name, "PED2_W") == 0 || strcmp(name, "PED2_WALK") == 0) return 11;
        if (strcmp(name, "PED2_DW") == 0 || strcmp(name, "PED2_DONT_WALK") == 0) return 12;

        // Numeric channel index (0..12) or physical GPIO pin number lookup
        char* endp = nullptr;
        long val = strtol(name, &endp, 10);
        if (endp && *endp == '\0' && endp != name) {
            if (val >= 0 && val < static_cast<long>(TrafficConfig::TOTAL_SIGNAL_OUTPUTS)) {
                return static_cast<int8_t>(val);
            }
            for (uint8_t i = 0; i < TrafficConfig::TOTAL_SIGNAL_OUTPUTS; ++i) {
                if (TrafficConfig::SIGNAL_OUTPUT_PINS[i] == static_cast<uint8_t>(val)) {
                    return static_cast<int8_t>(i);
                }
            }
        }
        return -2;
    }

    // Confirmed hardware pin configuration: Side Yellow is assigned to GPIO 12 (PIN_SIDE_YELLOW).
    void setUseSafeProposedSideYellowPin(bool /*useSafePin*/) {
        sideYellowPin_ = TrafficConfig::PIN_SIDE_YELLOW;
    }

    uint8_t getSideYellowPin() const { return sideYellowPin_; }
    TrafficConfig::OutputDriverType getVehicleDriverType() const { return vehicleDriverType_; }
    TrafficConfig::OutputDriverType getPedestrianDriverType() const { return pedDriverType_; }
    bool isVehicleDriverVerified() const { return vehicleDriverVerified_; }
    bool isPedestrianDriverVerified() const { return pedDriverVerified_; }

    // Layer 2: Compute whether the ESP32 GPIO pin should be driven HIGH (true) or LOW (false)
    // for a given logical lamp command and driver type.
    static bool computePinLevel(bool logicalLampOn,
                                TrafficConfig::OutputDriverType driverType,
                                bool driverVerified) {
        const bool effectiveOn = driverVerified ? logicalLampOn : false;
        switch (driverType) {
            case TrafficConfig::OutputDriverType::HIGH_SIDE_SOURCING_ACTIVE_LOW:
                // Inverting high-side driver or ULN2003A shunt with pull-up load:
                // GPIO HIGH = lamp OFF (ULN2003A shunts or high-side switch off)
                // GPIO LOW  = lamp ON
                return !effectiveOn;

            case TrafficConfig::OutputDriverType::DIRECT_ACTIVE_HIGH:
            case TrafficConfig::OutputDriverType::HIGH_SIDE_SOURCING_ACTIVE_HIGH:
            case TrafficConfig::OutputDriverType::LOW_SIDE_ULN2003A_ACTIVE_HIGH:
                // Direct active-HIGH GPIO, non-inverting high-side source, or ULN2003A low-side sink:
                // GPIO HIGH = lamp ON (for ULN2003A: output sinks current LOW to GND), GPIO LOW = lamp OFF
                return effectiveOn;

            case TrafficConfig::OutputDriverType::UNVERIFIED_SAFE_DISABLED:
            default:
                // Safe de-energized output state
                return false;
        }
    }

    // Layer 3: Compute whether the ULN2003A NPN Darlington channel is actively
    // sinking current to GND for a given ESP32 GPIO level and driver topology.
    static bool computeUlnSinkState(bool espGpioHigh, TrafficConfig::OutputDriverType driverType) {
        if (driverType == TrafficConfig::OutputDriverType::UNVERIFIED_SAFE_DISABLED ||
            driverType == TrafficConfig::OutputDriverType::DIRECT_ACTIVE_HIGH) {
            return false;
        }
        // A ULN2003A Darlington pair conducts (sinks current at OUTx to GND)
        // whenever its input pin INx is driven HIGH (3.3V) by the ESP32 GPIO.
        return espGpioHigh;
    }

    // Layer 4: Compute whether the physical traffic-light lamp is electrically
    // illuminated given the ESP32 GPIO state, ULN2003A sink state, and driver verification.
    static bool computeLampIlluminated(bool espGpioHigh,
                                       bool ulnSinkActive,
                                       TrafficConfig::OutputDriverType driverType,
                                       bool driverVerified) {
        if (!driverVerified || driverType == TrafficConfig::OutputDriverType::UNVERIFIED_SAFE_DISABLED) {
            return false;
        }
        switch (driverType) {
            case TrafficConfig::OutputDriverType::LOW_SIDE_ULN2003A_ACTIVE_HIGH:
            case TrafficConfig::OutputDriverType::HIGH_SIDE_SOURCING_ACTIVE_HIGH:
                return ulnSinkActive && espGpioHigh;
            case TrafficConfig::OutputDriverType::HIGH_SIDE_SOURCING_ACTIVE_LOW:
                return !espGpioHigh && !ulnSinkActive;
            case TrafficConfig::OutputDriverType::DIRECT_ACTIVE_HIGH:
                return espGpioHigh;
            default:
                return false;
        }
    }

    void initializeHardwareSafe() {
        getPinList(pins_);
        // Set all channels to their electrical OFF state BEFORE enabling OUTPUT mode
        LogicalSignalOutputs allLogicalOff = {
            false, false, false, false, false, false, false, false,
            false, false, false, false, false
        };
        computeElectricalStates(allLogicalOff, lastElectrical_);

#ifdef ARDUINO
        for (uint8_t i = 0; i < TrafficConfig::TOTAL_SIGNAL_OUTPUTS; ++i) {
            digitalWrite(pins_[i], lastElectrical_.pinHigh[i] ? HIGH : LOW);
            pinMode(pins_[i], OUTPUT);
            digitalWrite(pins_[i], lastElectrical_.pinHigh[i] ? HIGH : LOW);
        }
#endif
    }

    void forceAllSafeOff() {
        lampTestChannel_ = -1;
        LogicalSignalOutputs allLogicalOff = {
            false, false, false, false, false, false, false, false,
            false, false, false, false, false
        };
        applyLogicalOutputs(allLogicalOff);
    }

    // Apply logical signal outputs using turn-off-before-turn-on (break-before-make) sequencing
    void applyLogicalOutputs(const LogicalSignalOutputs& logical) {
        getPinList(pins_);
        const LogicalSignalOutputs effectiveLogical =
            (lampTestChannel_ >= 0 && lampTestChannel_ < static_cast<int8_t>(TrafficConfig::TOTAL_SIGNAL_OUTPUTS))
                ? buildSingleLampLogicalOutputs(lampTestChannel_)
                : logical;
        ElectricalPinStates target;
        computeElectricalStates(effectiveLogical, target);

#ifdef ARDUINO
        // Pass 1: De-energize any channel that is turning OFF first
        for (uint8_t i = 0; i < TrafficConfig::TOTAL_SIGNAL_OUTPUTS; ++i) {
            if (!target.channelEnergized[i]) {
                digitalWrite(pins_[i], target.pinHigh[i] ? HIGH : LOW);
            }
        }
        // Pass 2: Energize channels that are turning ON
        for (uint8_t i = 0; i < TrafficConfig::TOTAL_SIGNAL_OUTPUTS; ++i) {
            if (target.channelEnergized[i]) {
                digitalWrite(pins_[i], target.pinHigh[i] ? HIGH : LOW);
            }
        }
#endif
        lastElectrical_ = target;
    }

    void computeElectricalStates(const LogicalSignalOutputs& logical, ElectricalPinStates& out) const {
        uint8_t pinMap[TrafficConfig::TOTAL_SIGNAL_OUTPUTS];
        getPinList(pinMap);

        const bool vehLogical[9] = {
            logical.mainGreen, logical.mainYellow, logical.mainRed,
            logical.sideGreen, logical.sideYellow, logical.sideRed,
            logical.turnGreen, logical.turnYellow, logical.turnRed
        };
        for (uint8_t i = 0; i < 9; ++i) {
            const bool canEnergize = vehicleDriverVerified_ &&
                                     (vehicleDriverType_ != TrafficConfig::OutputDriverType::UNVERIFIED_SAFE_DISABLED);
            const bool pinHigh     = computePinLevel(vehLogical[i], vehicleDriverType_, canEnergize);
            const bool ulnSinking  = computeUlnSinkState(pinHigh, vehicleDriverType_);
            out.gpioPin[i]          = pinMap[i];
            out.logicalState[i]     = vehLogical[i];
            out.pinHigh[i]          = pinHigh;
            out.ulnSinkActive[i]    = ulnSinking;
            out.channelEnergized[i] = computeLampIlluminated(pinHigh, ulnSinking, vehicleDriverType_, canEnergize);
        }

        const bool pedLogical[4] = {
            logical.pedWalk,  logical.pedDontWalk,
            logical.ped2Walk, logical.ped2DontWalk
        };
        for (uint8_t j = 0; j < 4; ++j) {
            const uint8_t idx = 9 + j;
            const bool canEnergize = pedDriverVerified_ &&
                                     (pedDriverType_ != TrafficConfig::OutputDriverType::UNVERIFIED_SAFE_DISABLED);
            const bool pinHigh     = computePinLevel(pedLogical[j], pedDriverType_, canEnergize);
            const bool ulnSinking  = computeUlnSinkState(pinHigh, pedDriverType_);
            out.gpioPin[idx]          = pinMap[idx];
            out.logicalState[idx]     = pedLogical[j];
            out.pinHigh[idx]          = pinHigh;
            out.ulnSinkActive[idx]    = ulnSinking;
            out.channelEnergized[idx] = computeLampIlluminated(pinHigh, ulnSinking, pedDriverType_, canEnergize);
        }
    }

    const ElectricalPinStates& getLastElectricalStates() const { return lastElectrical_; }

    void getPinList(uint8_t outPins[TrafficConfig::TOTAL_SIGNAL_OUTPUTS]) const {
        outPins[0]  = TrafficConfig::PIN_MAIN_GREEN;     // GPIO 25
        outPins[1]  = TrafficConfig::PIN_MAIN_YELLOW;    // GPIO 26
        outPins[2]  = TrafficConfig::PIN_MAIN_RED;       // GPIO 27
        outPins[3]  = TrafficConfig::PIN_SIDE_GREEN;     // GPIO 13 (PIN_SIDE_GREEN)
        outPins[4]  = sideYellowPin_;                    // GPIO 12 (PIN_SIDE_YELLOW)
        outPins[5]  = TrafficConfig::PIN_SIDE_RED;       // GPIO 14 (PIN_SIDE_RED)
        outPins[6]  = TrafficConfig::PIN_TURN_GREEN;     // GPIO 21
        outPins[7]  = TrafficConfig::PIN_TURN_YELLOW;    // GPIO 23
        outPins[8]  = TrafficConfig::PIN_TURN_RED;       // GPIO 22
        outPins[9]  = TrafficConfig::PIN_PED1_WALK;      // GPIO 19
        outPins[10] = TrafficConfig::PIN_PED1_DONT_WALK; // GPIO 18
        outPins[11] = TrafficConfig::PIN_PED2_WALK;      // GPIO 16
        outPins[12] = TrafficConfig::PIN_PED2_DONT_WALK; // GPIO 17
    }

    int8_t findChannelByGpio(uint8_t gpioPin) const {
        uint8_t pinMap[TrafficConfig::TOTAL_SIGNAL_OUTPUTS];
        getPinList(pinMap);
        for (uint8_t i = 0; i < TrafficConfig::TOTAL_SIGNAL_OUTPUTS; ++i) {
            if (pinMap[i] == gpioPin) {
                return static_cast<int8_t>(i);
            }
        }
        return -1;
    }

    bool isLogicalSignalOnForGpio(uint8_t gpioPin, const ElectricalPinStates& st) const {
        const int8_t idx = findChannelByGpio(gpioPin);
        return (idx >= 0) ? st.logicalState[idx] : false;
    }

    bool isGpioDrivenHigh(uint8_t gpioPin, const ElectricalPinStates& st) const {
        const int8_t idx = findChannelByGpio(gpioPin);
        return (idx >= 0) ? st.pinHigh[idx] : false;
    }

    bool isUlnSinkActiveForGpio(uint8_t gpioPin, const ElectricalPinStates& st) const {
        const int8_t idx = findChannelByGpio(gpioPin);
        return (idx >= 0) ? st.ulnSinkActive[idx] : false;
    }

    bool isLampEnergizedForGpio(uint8_t gpioPin, const ElectricalPinStates& st) const {
        const int8_t idx = findChannelByGpio(gpioPin);
        return (idx >= 0) ? st.channelEnergized[idx] : false;
    }

    static const char* driverTypeToString(TrafficConfig::OutputDriverType t) {
        switch (t) {
            case TrafficConfig::OutputDriverType::UNVERIFIED_SAFE_DISABLED:      return "UNVERIFIED_SAFE_DISABLED";
            case TrafficConfig::OutputDriverType::DIRECT_ACTIVE_HIGH:            return "DIRECT_ACTIVE_HIGH";
            case TrafficConfig::OutputDriverType::HIGH_SIDE_SOURCING_ACTIVE_HIGH:return "HIGH_SIDE_SOURCING_ACTIVE_HIGH";
            case TrafficConfig::OutputDriverType::HIGH_SIDE_SOURCING_ACTIVE_LOW: return "HIGH_SIDE_SOURCING_ACTIVE_LOW";
            case TrafficConfig::OutputDriverType::LOW_SIDE_ULN2003A_ACTIVE_HIGH: return "LOW_SIDE_ULN2003A_ACTIVE_HIGH";
            default:                                                             return "UNKNOWN";
        }
    }

private:
    void clearElectricalStates() {
        getPinList(pins_);
        for (uint8_t i = 0; i < TrafficConfig::TOTAL_SIGNAL_OUTPUTS; ++i) {
            lastElectrical_.gpioPin[i]          = pins_[i];
            lastElectrical_.logicalState[i]     = false;
            lastElectrical_.pinHigh[i]          = false;
            lastElectrical_.ulnSinkActive[i]    = false;
            lastElectrical_.channelEnergized[i] = false;
        }
    }

    TrafficConfig::OutputDriverType vehicleDriverType_;
    TrafficConfig::OutputDriverType pedDriverType_;
    bool                            vehicleDriverVerified_;
    bool                            pedDriverVerified_;
    uint8_t                         sideYellowPin_;
    int8_t                          lampTestChannel_;
    uint8_t                         pins_[TrafficConfig::TOTAL_SIGNAL_OUTPUTS];
    ElectricalPinStates             lastElectrical_;
};

#endif // SIGNAL_OUTPUT_DRIVER_H
