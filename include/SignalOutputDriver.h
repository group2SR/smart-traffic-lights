#ifndef SIGNAL_OUTPUT_DRIVER_H
#define SIGNAL_OUTPUT_DRIVER_H

#include <stdint.h>
#include "TrafficConfig.h"

#ifdef ARDUINO
#include <Arduino.h>
#endif

// ============================================================================
// SIGNAL OUTPUT DRIVER (Hardware Abstraction Layer)
// ============================================================================
// Separates logical traffic signal state from electrical GPIO control.
//
// CRITICAL HARDWARE SAFEGUARDS:
// 1. The 3 vehicle RYG modules are 5V common-ground (common-cathode) modules:
//    applying +5V to R, Y, or G illuminates the lamp.
// 2. ULN2003A is a low-side current-sinking Darlington driver and CANNOT source
//    +5V to common-ground vehicle modules.
// 3. Therefore, vehicle output driver type and polarity are configurable, and
//    physical vehicle output actuation is gated by vehicleDriverVerified_
//    (default false) until a suitable high-side sourcing driver is verified.
// 4. Pedestrian LEDs have independent anodes/cathodes and may use ULN2003A
//    as a low-side sink with common-anode (+5V) wiring and current-limiting
//    resistors.
// 5. Never assumes "GPIO LOW == safe lamp OFF". Electrical OFF and ON levels
//    are derived from each group's configured OutputDriverType.

struct LogicalSignalOutputs {
    // Core v1.1.0 fields (1..8) — preserved in exact order for aggregate compatibility
    bool mainGreen;
    bool mainYellow;
    bool mainRed;
    bool sideGreen;
    bool sideYellow;
    bool sideRed;
    bool pedWalk;       // Pedestrian Group 1 GREEN (WALK)
    bool pedDontWalk;   // Pedestrian Group 1 RED (DON'T WALK)

    // Extended v2.0.0 fields (9..13)
    bool turnGreen;
    bool turnYellow;
    bool turnRed;
    bool ped2Walk;      // Pedestrian Group 2 GREEN (WALK)
    bool ped2DontWalk;  // Pedestrian Group 2 RED (DON'T WALK)
};

struct ElectricalPinStates {
    bool pinHigh[TrafficConfig::TOTAL_SIGNAL_OUTPUTS];
    bool channelEnergized[TrafficConfig::TOTAL_SIGNAL_OUTPUTS];
};

class SignalOutputDriver {
public:
    SignalOutputDriver()
        : vehicleDriverType_(TrafficConfig::OutputDriverType::UNVERIFIED_SAFE_DISABLED),
          pedDriverType_(TrafficConfig::OutputDriverType::LOW_SIDE_ULN2003A_ACTIVE_HIGH),
          vehicleDriverVerified_(TrafficConfig::VEHICLE_HIGH_SIDE_DRIVER_VERIFIED),
          pedDriverVerified_(TrafficConfig::PED_ULN2003A_DRIVER_VERIFIED),
          sideYellowPin_(TrafficConfig::PIN_SIDE_YELLOW) {
        clearElectricalStates();
    }

    void setVehicleDriverConfig(TrafficConfig::OutputDriverType type, bool verified) {
        vehicleDriverType_     = type;
        vehicleDriverVerified_ = verified;
    }

    void setPedestrianDriverConfig(TrafficConfig::OutputDriverType type, bool verified) {
        pedDriverType_     = type;
        pedDriverVerified_ = verified;
    }

    void setUseSafeProposedSideYellowPin(bool useSafePin5) {
        sideYellowPin_ = useSafePin5 ? TrafficConfig::PIN_SIDE_YELLOW_SAFE_PROPOSED
                                     : TrafficConfig::PIN_SIDE_YELLOW;
    }

    uint8_t getSideYellowPin() const { return sideYellowPin_; }
    TrafficConfig::OutputDriverType getVehicleDriverType() const { return vehicleDriverType_; }
    TrafficConfig::OutputDriverType getPedestrianDriverType() const { return pedDriverType_; }
    bool isVehicleDriverVerified() const { return vehicleDriverVerified_; }
    bool isPedestrianDriverVerified() const { return pedDriverVerified_; }

    // Compute whether the ESP32 GPIO pin should be driven HIGH (true) or LOW (false)
    // for a given logical lamp command and driver type.
    static bool computePinLevel(bool logicalLampOn,
                                TrafficConfig::OutputDriverType driverType,
                                bool driverVerified) {
        const bool effectiveOn = driverVerified ? logicalLampOn : false;
        switch (driverType) {
            case TrafficConfig::OutputDriverType::HIGH_SIDE_SOURCING_ACTIVE_LOW:
                // Inverting high-side driver (e.g. direct P-MOSFET / PNP):
                // HIGH = OFF, LOW = ON
                return !effectiveOn;

            case TrafficConfig::OutputDriverType::DIRECT_LOGIC_ACTIVE_HIGH:
            case TrafficConfig::OutputDriverType::HIGH_SIDE_SOURCING_ACTIVE_HIGH:
            case TrafficConfig::OutputDriverType::LOW_SIDE_ULN2003A_ACTIVE_HIGH:
                // Non-inverting driver (or ULN2003A low-side sink with common-anode LED):
                // HIGH = ON, LOW = OFF
                return effectiveOn;

            case TrafficConfig::OutputDriverType::UNVERIFIED_SAFE_DISABLED:
            default:
                // Safe de-energized output state
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
        LogicalSignalOutputs allLogicalOff = {
            false, false, false, false, false, false, false, false,
            false, false, false, false, false
        };
        applyLogicalOutputs(allLogicalOff);
    }

    // Apply logical signal outputs using turn-off-before-turn-on sequencing
    void applyLogicalOutputs(const LogicalSignalOutputs& logical) {
        getPinList(pins_);
        ElectricalPinStates target;
        computeElectricalStates(logical, target);

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
        const bool vehLogical[9] = {
            logical.mainGreen, logical.mainYellow, logical.mainRed,
            logical.sideGreen, logical.sideYellow, logical.sideRed,
            logical.turnGreen, logical.turnYellow, logical.turnRed
        };
        for (uint8_t i = 0; i < 9; ++i) {
            const bool canEnergize = vehicleDriverVerified_ &&
                                     (vehicleDriverType_ != TrafficConfig::OutputDriverType::UNVERIFIED_SAFE_DISABLED);
            out.channelEnergized[i] = canEnergize && vehLogical[i];
            out.pinHigh[i]          = computePinLevel(vehLogical[i], vehicleDriverType_, canEnergize);
        }

        const bool pedLogical[4] = {
            logical.pedWalk,  logical.pedDontWalk,
            logical.ped2Walk, logical.ped2DontWalk
        };
        for (uint8_t j = 0; j < 4; ++j) {
            const uint8_t idx = 9 + j;
            const bool canEnergize = pedDriverVerified_ &&
                                     (pedDriverType_ != TrafficConfig::OutputDriverType::UNVERIFIED_SAFE_DISABLED);
            out.channelEnergized[idx] = canEnergize && pedLogical[j];
            out.pinHigh[idx]          = computePinLevel(pedLogical[j], pedDriverType_, canEnergize);
        }
    }

    const ElectricalPinStates& getLastElectricalStates() const { return lastElectrical_; }

    void getPinList(uint8_t outPins[TrafficConfig::TOTAL_SIGNAL_OUTPUTS]) const {
        outPins[0]  = TrafficConfig::PIN_MAIN_GREEN;
        outPins[1]  = TrafficConfig::PIN_MAIN_YELLOW;
        outPins[2]  = TrafficConfig::PIN_MAIN_RED;
        outPins[3]  = TrafficConfig::PIN_SIDE_GREEN;
        outPins[4]  = sideYellowPin_;
        outPins[5]  = TrafficConfig::PIN_SIDE_RED;
        outPins[6]  = TrafficConfig::PIN_TURN_GREEN;
        outPins[7]  = TrafficConfig::PIN_TURN_YELLOW;
        outPins[8]  = TrafficConfig::PIN_TURN_RED;
        outPins[9]  = TrafficConfig::PIN_PED1_WALK;
        outPins[10] = TrafficConfig::PIN_PED1_DONT_WALK;
        outPins[11] = TrafficConfig::PIN_PED2_WALK;
        outPins[12] = TrafficConfig::PIN_PED2_DONT_WALK;
    }

    static const char* driverTypeToString(TrafficConfig::OutputDriverType t) {
        switch (t) {
            case TrafficConfig::OutputDriverType::UNVERIFIED_SAFE_DISABLED:      return "UNVERIFIED_SAFE_DISABLED";
            case TrafficConfig::OutputDriverType::DIRECT_LOGIC_ACTIVE_HIGH:      return "DIRECT_LOGIC_ACTIVE_HIGH";
            case TrafficConfig::OutputDriverType::HIGH_SIDE_SOURCING_ACTIVE_HIGH:return "HIGH_SIDE_SOURCING_ACTIVE_HIGH";
            case TrafficConfig::OutputDriverType::HIGH_SIDE_SOURCING_ACTIVE_LOW: return "HIGH_SIDE_SOURCING_ACTIVE_LOW";
            case TrafficConfig::OutputDriverType::LOW_SIDE_ULN2003A_ACTIVE_HIGH: return "LOW_SIDE_ULN2003A_ACTIVE_HIGH";
            default:                                                             return "UNKNOWN";
        }
    }

private:
    void clearElectricalStates() {
        for (uint8_t i = 0; i < TrafficConfig::TOTAL_SIGNAL_OUTPUTS; ++i) {
            pins_[i] = 0;
            lastElectrical_.pinHigh[i]          = false;
            lastElectrical_.channelEnergized[i] = false;
        }
    }

    TrafficConfig::OutputDriverType vehicleDriverType_;
    TrafficConfig::OutputDriverType pedDriverType_;
    bool                            vehicleDriverVerified_;
    bool                            pedDriverVerified_;
    uint8_t                         sideYellowPin_;
    uint8_t                         pins_[TrafficConfig::TOTAL_SIGNAL_OUTPUTS];
    ElectricalPinStates             lastElectrical_;
};

#endif // SIGNAL_OUTPUT_DRIVER_H

