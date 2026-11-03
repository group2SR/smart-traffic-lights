#include <Arduino.h>
#include <unity.h>
#include "TrafficConfig.h"
#include "TrafficController.h"
#include "SerialCommandHandler.h"

void setUp(void) {}
void tearDown(void) {}

// ============================================================================
// PART I: PRESERVED BASELINE REGRESSION TESTS (Tests 1 .. 13)
// ============================================================================

// 1. Test Startup to ALL_RED and ALL_RED to MAIN_GREEN
void test_startup_to_all_red_to_main_green(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    TEST_ASSERT_EQUAL(TrafficState::STARTUP_SAFE, ctrl.getState());
    TEST_ASSERT_EQUAL(OperatingMode::HARDWARE_READY, ctrl.getMode());

    // During STARTUP_SAFE, all outputs must be LOW
    SignalOutputs out0 = ctrl.getOutputs();
    TEST_ASSERT_FALSE(out0.mainGreen);
    TEST_ASSERT_FALSE(out0.mainYellow);
    TEST_ASSERT_FALSE(out0.mainRed);
    TEST_ASSERT_FALSE(out0.sideGreen);
    TEST_ASSERT_FALSE(out0.sideYellow);
    TEST_ASSERT_FALSE(out0.sideRed);
    TEST_ASSERT_FALSE(out0.pedWalk);
    TEST_ASSERT_FALSE(out0.pedDontWalk);
    TEST_ASSERT_FALSE(out0.turnGreen);
    TEST_ASSERT_FALSE(out0.turnYellow);
    TEST_ASSERT_FALSE(out0.turnRed);
    TEST_ASSERT_FALSE(out0.ped2Walk);
    TEST_ASSERT_FALSE(out0.ped2DontWalk);

    // Before STARTUP_SAFE_MS (1000 ms)
    ctrl.update(999);
    TEST_ASSERT_EQUAL(TrafficState::STARTUP_SAFE, ctrl.getState());

    // At 1000 ms -> ALL_RED
    ctrl.update(1000);
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());
    SignalOutputs outAllRed = ctrl.getOutputs();
    TEST_ASSERT_TRUE(outAllRed.mainRed);
    TEST_ASSERT_TRUE(outAllRed.sideRed);
    TEST_ASSERT_TRUE(outAllRed.turnRed);
    TEST_ASSERT_TRUE(outAllRed.pedDontWalk);
    TEST_ASSERT_TRUE(outAllRed.ped2DontWalk);
    TEST_ASSERT_FALSE(outAllRed.mainGreen);
    TEST_ASSERT_FALSE(outAllRed.sideGreen);
    TEST_ASSERT_FALSE(outAllRed.turnGreen);
    TEST_ASSERT_FALSE(outAllRed.pedWalk);
    TEST_ASSERT_FALSE(outAllRed.ped2Walk);

    // Before ALL_RED_MS (1000 ms elapsed from 1000 -> 1999)
    ctrl.update(1999);
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());

    // At 2000 ms -> MAIN_GREEN
    ctrl.update(2000);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());
    SignalOutputs outMain = ctrl.getOutputs();
    TEST_ASSERT_TRUE(outMain.mainGreen);
    TEST_ASSERT_TRUE(outMain.sideRed);
    TEST_ASSERT_TRUE(outMain.turnRed);
    TEST_ASSERT_TRUE(outMain.pedDontWalk);
    TEST_ASSERT_TRUE(outMain.ped2DontWalk);
}

// 2. Test Side-road demand while MAIN_GREEN is active, Minimum green enforcement,
//    Main-road yellow and all-red transitions, and Side-road green + early termination
void test_side_road_service_and_early_termination(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.setSimulationMode(0, true);
    ctrl.update(1000); // -> ALL_RED
    ctrl.update(2000); // -> MAIN_GREEN (starts at 2000 ms)

    // Register side-road vehicle demand at 2500 ms (only 500 ms into MAIN_GREEN)
    TEST_ASSERT_TRUE(ctrl.setSimVehicleDemand(2500, true));
    TEST_ASSERT_TRUE(ctrl.hasSideVehicleDemand());

    // Minimum main-road green (4000 ms) must be enforced until 6000 ms
    ctrl.update(3000);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());
    ctrl.update(5999);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());

    // At 6000 ms -> MAIN_YELLOW (2000 ms duration)
    ctrl.update(6000);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_YELLOW, ctrl.getState());
    ctrl.update(7999);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_YELLOW, ctrl.getState());

    // At 8000 ms -> ALL_RED (1000 ms duration)
    ctrl.update(8000);
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());
    ctrl.update(8999);
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());

    // At 9000 ms -> SIDE_GREEN
    ctrl.update(9000);
    TEST_ASSERT_EQUAL(TrafficState::SIDE_GREEN, ctrl.getState());
    SignalOutputs outSide = ctrl.getOutputs();
    TEST_ASSERT_TRUE(outSide.sideGreen);
    TEST_ASSERT_TRUE(outSide.mainRed);
    TEST_ASSERT_TRUE(outSide.turnRed);
    TEST_ASSERT_TRUE(outSide.pedDontWalk);

    // Clear vehicle demand at 9500 ms (before 2000 ms min green is up)
    ctrl.setSimVehicleDemand(9500, false);
    ctrl.update(11000); // Min green (2000 ms) reached at 11000, gap elapsed is 1500 ms (< 2000 ms)
    TEST_ASSERT_EQUAL(TrafficState::SIDE_GREEN, ctrl.getState());

    // At 11500 ms: elapsed = 2500 ms (>= 2000 ms min green) AND gap = 2000 ms -> SIDE_YELLOW
    ctrl.update(11500);
    TEST_ASSERT_EQUAL(TrafficState::SIDE_YELLOW, ctrl.getState());

    // SIDE_YELLOW (2000 ms) -> ALL_RED at 13500 ms -> MAIN_GREEN at 14500 ms
    ctrl.update(13500);
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());
    ctrl.update(14500);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());
}

// 3. Test Side-road maximum green enforcement (8000 ms)
void test_side_road_max_green(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.setSimulationMode(0, true);
    ctrl.update(1000); // -> ALL_RED
    ctrl.update(2000); // -> MAIN_GREEN
    ctrl.setSimVehicleDemand(2000, true);
    ctrl.update(6000); // -> MAIN_YELLOW
    ctrl.update(8000); // -> ALL_RED
    ctrl.update(9000); // -> SIDE_GREEN (vehicle stays continuously ON)

    ctrl.update(16999); // 7999 ms into SIDE_GREEN
    TEST_ASSERT_EQUAL(TrafficState::SIDE_GREEN, ctrl.getState());

    // At 17000 ms (8000 ms max green) -> forced to SIDE_YELLOW despite continuous demand
    ctrl.update(17000);
    TEST_ASSERT_EQUAL(TrafficState::SIDE_YELLOW, ctrl.getState());
    ctrl.update(19000);
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());
    ctrl.update(20000);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());
}

// 4. Test Pedestrian request latching, service, and return to main-road priority
void test_pedestrian_service(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.setSimulationMode(0, true);
    ctrl.update(1000); // -> ALL_RED
    ctrl.update(2000); // -> MAIN_GREEN

    // Request pedestrian at 3000 ms -> immediately latched
    TEST_ASSERT_TRUE(ctrl.triggerSimPedestrianRequest(3000));
    TEST_ASSERT_TRUE(ctrl.hasPedestrianDemand());

    // Does not interrupt minimum main green (until 6000 ms)
    ctrl.update(5999);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());
    TEST_ASSERT_TRUE(ctrl.hasPedestrianDemand());

    // 6000 ms -> MAIN_YELLOW -> 8000 ms ALL_RED -> 9000 ms PED_WALK
    ctrl.update(6000);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_YELLOW, ctrl.getState());
    ctrl.update(8000);
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());
    ctrl.update(9000);
    TEST_ASSERT_EQUAL(TrafficState::PED_WALK, ctrl.getState());

    // Request remains latched during WALK until completion
    TEST_ASSERT_TRUE(ctrl.hasPedestrianDemand());
    SignalOutputs outWalk = ctrl.getOutputs();
    TEST_ASSERT_TRUE(outWalk.pedWalk);
    TEST_ASSERT_FALSE(outWalk.pedDontWalk);
    TEST_ASSERT_TRUE(outWalk.mainRed);
    TEST_ASSERT_TRUE(outWalk.sideRed);
    TEST_ASSERT_TRUE(outWalk.turnRed);
    TEST_ASSERT_FALSE(outWalk.mainGreen);
    TEST_ASSERT_FALSE(outWalk.sideGreen);
    TEST_ASSERT_FALSE(outWalk.turnGreen);

    // WALK lasts 5000 ms (9000 -> 14000 ms)
    ctrl.update(13999);
    TEST_ASSERT_EQUAL(TrafficState::PED_WALK, ctrl.getState());

    ctrl.update(14000);
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());
    TEST_ASSERT_FALSE(ctrl.hasPedestrianDemand()); // Unlatched upon completion

    ctrl.update(15000);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());
}

// 5. Test Simultaneous pedestrian and vehicle requests (deterministic arbitration without starvation)
void test_simultaneous_pedestrian_and_vehicle_requests(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.setSimulationMode(0, true);
    ctrl.update(1000); // -> ALL_RED
    ctrl.update(2000); // -> MAIN_GREEN

    // Both requests arrive during MAIN_GREEN
    ctrl.setSimVehicleDemand(2500, true);
    ctrl.triggerSimPedestrianRequest(2600);

    ctrl.update(6000); // -> MAIN_YELLOW
    ctrl.update(8000); // -> ALL_RED
    ctrl.update(9000); // First serves PED_WALK (since lastServedNonMain_ was not PED_WALK)
    TEST_ASSERT_EQUAL(TrafficState::PED_WALK, ctrl.getState());
    TEST_ASSERT_TRUE(ctrl.hasSideVehicleDemand()); // Side demand still preserved!

    ctrl.update(14000); // PED_WALK done -> ALL_RED
    ctrl.update(15000); // Restores MAIN_GREEN first to prevent main-road starvation
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());

    // Suppose another pedestrian request arrives immediately while side demand is still waiting
    ctrl.triggerSimPedestrianRequest(15500);

    // After MAIN_GREEN min duration (15000 + 4000 = 19000 ms), round-robin selects SIDE_ROAD next!
    ctrl.update(19000); // -> MAIN_YELLOW
    ctrl.update(21000); // -> ALL_RED
    ctrl.update(22000); // -> SIDE_GREEN
    TEST_ASSERT_EQUAL(TrafficState::SIDE_GREEN, ctrl.getState());
}

// 6. Test Simulation mode rejection of inappropriate commands & SIM OFF cleanup
void test_simulation_mode_rejection_and_sim_off_cleanup(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    TEST_ASSERT_EQUAL(OperatingMode::HARDWARE_READY, ctrl.getMode());

    // Simulation-only commands must be rejected in HARDWARE_READY
    TEST_ASSERT_FALSE(ctrl.setSimVehicleDemand(500, true));
    TEST_ASSERT_FALSE(ctrl.triggerSimPedestrianRequest(500));
    TEST_ASSERT_FALSE(ctrl.clearSimulatedDemand(500));
    TEST_ASSERT_FALSE(ctrl.hasSideVehicleDemand());
    TEST_ASSERT_FALSE(ctrl.hasPedestrianDemand());

    // Advance to SIDE_GREEN in simulation mode
    ctrl.update(1000); // -> ALL_RED
    ctrl.update(2000); // -> MAIN_GREEN
    ctrl.setSimulationMode(2000, true);
    ctrl.setSimVehicleDemand(2100, true);
    ctrl.triggerSimPedestrianRequest(2200);
    ctrl.update(6000); // -> MAIN_YELLOW
    ctrl.update(8000); // -> ALL_RED
    ctrl.update(9000); // -> PED_WALK

    // Switch SIM OFF during PED_WALK -> discards simulated demands and transitions via ALL_RED
    ctrl.setSimulationMode(9500, false);
    TEST_ASSERT_EQUAL(OperatingMode::HARDWARE_READY, ctrl.getMode());
    TEST_ASSERT_FALSE(ctrl.hasSideVehicleDemand());
    TEST_ASSERT_FALSE(ctrl.hasPedestrianDemand());
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());

    ctrl.update(10500);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());
}

// 7. Test Reset during every normal phase without momentary conflicting greens
void test_reset_during_every_normal_phase(void) {
    const TrafficState testPhases[] = {
        TrafficState::ALL_RED,
        TrafficState::MAIN_GREEN,
        TrafficState::MAIN_YELLOW,
        TrafficState::SIDE_GREEN,
        TrafficState::SIDE_YELLOW,
        TrafficState::PED_WALK
    };

    for (TrafficState targetPhase : testPhases) {
        TrafficController ctrl;
        ctrl.begin(0);
        ctrl.setSimulationMode(0, true);
        uint32_t t = 1000;
        ctrl.update(t); // ALL_RED

        if (targetPhase != TrafficState::ALL_RED) {
            t += 1000;
            ctrl.update(t); // MAIN_GREEN
        }
        if (targetPhase == TrafficState::MAIN_YELLOW ||
            targetPhase == TrafficState::SIDE_GREEN ||
            targetPhase == TrafficState::SIDE_YELLOW) {
            ctrl.setSimVehicleDemand(t, true);
            t += 4000;
            ctrl.update(t); // MAIN_YELLOW
        }
        if (targetPhase == TrafficState::SIDE_GREEN || targetPhase == TrafficState::SIDE_YELLOW) {
            t += 2000;
            ctrl.update(t); // ALL_RED
            t += 1000;
            ctrl.update(t); // SIDE_GREEN
        }
        if (targetPhase == TrafficState::SIDE_YELLOW) {
            t += 8000;
            ctrl.update(t); // SIDE_YELLOW
        }
        if (targetPhase == TrafficState::PED_WALK) {
            ctrl.triggerSimPedestrianRequest(t);
            t += 4000;
            ctrl.update(t); // MAIN_YELLOW
            t += 2000;
            ctrl.update(t); // ALL_RED
            t += 1000;
            ctrl.update(t); // PED_WALK
        }

        TEST_ASSERT_EQUAL(targetPhase, ctrl.getState());

        // Perform logical reset
        ctrl.resetLogical(t + 100, "UNIT_TEST_RESET");
        TEST_ASSERT_EQUAL(TrafficState::STARTUP_SAFE, ctrl.getState());
        SignalOutputs outReset = ctrl.getOutputs();
        const char* violation = nullptr;
        TEST_ASSERT_TRUE(TrafficController::verifySafetyInvariants(ctrl.getState(), outReset, &violation));
        TEST_ASSERT_FALSE(outReset.mainGreen);
        TEST_ASSERT_FALSE(outReset.sideGreen);
        TEST_ASSERT_FALSE(outReset.turnGreen);
        TEST_ASSERT_FALSE(outReset.pedWalk);
    }
}

// 8. Test millis() rollover-safe timing across 0xFFFFFFFF -> 0x00000000 wraparound
void test_millis_rollover_safety(void) {
    TrafficController ctrl;
    const uint32_t startNearWrap = 0xFFFFFFFFUL - 400UL; // 400 ms before 32-bit rollover
    ctrl.begin(startNearWrap);
    TEST_ASSERT_EQUAL(TrafficState::STARTUP_SAFE, ctrl.getState());

    ctrl.update(startNearWrap + 300UL);
    TEST_ASSERT_EQUAL(TrafficState::STARTUP_SAFE, ctrl.getState());

    ctrl.update(startNearWrap + 999UL);
    TEST_ASSERT_EQUAL(TrafficState::STARTUP_SAFE, ctrl.getState());

    ctrl.update(startNearWrap + 1000UL);
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());

    ctrl.update(startNearWrap + 2000UL);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());
}

// 9. Test Prevention of conflicting greens, vehicle green during WALK, and Fault-state output behavior
void test_safety_invariants_and_fault_safe(void) {
    const TrafficState allStates[] = {
        TrafficState::STARTUP_SAFE,
        TrafficState::ALL_RED,
        TrafficState::MAIN_GREEN,
        TrafficState::MAIN_YELLOW,
        TrafficState::SIDE_GREEN,
        TrafficState::SIDE_YELLOW,
        TrafficState::TURN_GREEN,
        TrafficState::TURN_YELLOW,
        TrafficState::PED_WALK,
        TrafficState::FAULT_SAFE
    };

    for (TrafficState s : allStates) {
        SignalOutputs out = TrafficController::deriveOutputsForState(s);
        const char* reason = nullptr;
        TEST_ASSERT_TRUE(TrafficController::verifySafetyInvariants(s, out, &reason));
    }

    // Verify conflicting vehicle greens are caught
    SignalOutputs badConflictGreens = {true, false, false, true, false, false, false, true};
    const char* reason = nullptr;
    TEST_ASSERT_FALSE(TrafficController::verifySafetyInvariants(TrafficState::MAIN_GREEN, badConflictGreens, &reason));

    // Verify vehicle green during pedestrian WALK is caught
    SignalOutputs badWalkWithGreen = {true, false, false, false, false, true, true, false};
    TEST_ASSERT_FALSE(TrafficController::verifySafetyInvariants(TrafficState::PED_WALK, badWalkWithGreen, &reason));

    // Verify FAULT_SAFE state behavior
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.update(1000);
    ctrl.update(2000);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());

    ctrl.triggerFault(2500, "SIMULATED_INTERNAL_FAULT");
    TEST_ASSERT_EQUAL(TrafficState::FAULT_SAFE, ctrl.getState());
    SignalOutputs faultOut = ctrl.getOutputs();
    TEST_ASSERT_FALSE(faultOut.mainGreen);
    TEST_ASSERT_FALSE(faultOut.mainYellow);
    TEST_ASSERT_FALSE(faultOut.mainRed);
    TEST_ASSERT_FALSE(faultOut.sideGreen);
    TEST_ASSERT_FALSE(faultOut.sideYellow);
    TEST_ASSERT_FALSE(faultOut.sideRed);
    TEST_ASSERT_FALSE(faultOut.turnGreen);
    TEST_ASSERT_FALSE(faultOut.turnYellow);
    TEST_ASSERT_FALSE(faultOut.turnRed);
    TEST_ASSERT_FALSE(faultOut.pedWalk);
    TEST_ASSERT_FALSE(faultOut.pedDontWalk);
    TEST_ASSERT_FALSE(faultOut.ped2Walk);
    TEST_ASSERT_FALSE(faultOut.ped2DontWalk);

    // Ensure FAULT_SAFE stops normal phase progression
    ctrl.update(10000);
    TEST_ASSERT_EQUAL(TrafficState::FAULT_SAFE, ctrl.getState());
}

// 10. Test Physical button debounce (50 ms) and hold suppression
void test_physical_button_debounce_and_hold_suppression(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.update(1000);
    ctrl.update(2000);

    ctrl.processPhysicalInputs(2000, true, true);
    TEST_ASSERT_FALSE(ctrl.hasPedestrianDemand());

    // Noisy glitch (< 50 ms LOW)
    ctrl.processPhysicalInputs(2010, false, true);
    ctrl.processPhysicalInputs(2040, false, true);
    ctrl.processPhysicalInputs(2045, true, true);
    ctrl.processPhysicalInputs(2100, true, true);
    TEST_ASSERT_FALSE(ctrl.hasPedestrianDemand());

    // Valid press (>= 50 ms LOW)
    ctrl.processPhysicalInputs(2200, false, true);
    ctrl.processPhysicalInputs(2249, false, true);
    TEST_ASSERT_FALSE(ctrl.hasPedestrianDemand());
    ctrl.processPhysicalInputs(2250, false, true);
    TEST_ASSERT_TRUE(ctrl.hasPedestrianDemand());

    // Serve the pedestrian request while the button remains continuously held LOW
    ctrl.update(6000);  // -> MAIN_YELLOW
    ctrl.update(8000);  // -> ALL_RED
    ctrl.update(9000);  // -> PED_WALK
    ctrl.update(14000); // -> ALL_RED (pedRequestLatched_ cleared!)
    TEST_ASSERT_FALSE(ctrl.hasPedestrianDemand());

    // Button is STILL held LOW at 14500 ms -> must NOT generate another request until released!
    ctrl.processPhysicalInputs(14500, false, true);
    ctrl.processPhysicalInputs(14600, false, true);
    TEST_ASSERT_FALSE(ctrl.hasPedestrianDemand());

    // Release button stably (>= 50 ms HIGH) then press again
    ctrl.processPhysicalInputs(15000, true, true);
    ctrl.processPhysicalInputs(15060, true, true);
    ctrl.processPhysicalInputs(15100, false, true);
    ctrl.processPhysicalInputs(15160, false, true);
    TEST_ASSERT_TRUE(ctrl.hasPedestrianDemand());
}

// 11. Test No demand detected (Main road stays GREEN indefinitely without false transitions)
void test_no_demand_continuous_main_green(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.update(1000); // -> ALL_RED
    ctrl.update(2000); // -> MAIN_GREEN
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());

    for (uint32_t t = 3000; t <= 120000; t += 1000) {
        HardwareInputs idleInputs = {true, true, true};
        ctrl.processHardwareAdapter(t, idleInputs);
        ctrl.update(t);
        TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());
        TEST_ASSERT_FALSE(ctrl.hasSideVehicleDemand());
        TEST_ASSERT_FALSE(ctrl.hasPedestrianDemand());
    }
}

// 12. Test Invalid timing configuration falls back to documented safe defaults
void test_invalid_timing_configuration_fallback(void) {
    TrafficController ctrl;
    ctrl.begin(0);

    TrafficConfig::TimingConfig badCfg = {
        0,       // startupSafeMs (invalid: < 500 ms)
        100,     // allRedMs (invalid: < 500 ms)
        0,       // mainMinGreenMs (invalid)
        250000,  // mainYellowMs (invalid: > 120000 ms)
        6000,    // sideMinGreenMs
        3000,    // sideMaxGreenMs (invalid: < sideMinGreenMs)
        0,       // sideClearGapMs (invalid)
        50,      // sideYellowMs (invalid)
        999999   // pedWalkMs (invalid)
    };

    TEST_ASSERT_FALSE(ctrl.setTimingConfig(0, badCfg));

    const TrafficConfig::TimingConfig& applied = ctrl.getTimingConfig();
    TEST_ASSERT_EQUAL_UINT32(TrafficConfig::STARTUP_SAFE_MS,   applied.startupSafeMs);
    TEST_ASSERT_EQUAL_UINT32(TrafficConfig::ALL_RED_MS,        applied.allRedMs);
    TEST_ASSERT_EQUAL_UINT32(TrafficConfig::MAIN_MIN_GREEN_MS, applied.mainMinGreenMs);
    TEST_ASSERT_EQUAL_UINT32(TrafficConfig::MAIN_YELLOW_MS,    applied.mainYellowMs);
    TEST_ASSERT_EQUAL_UINT32(TrafficConfig::SIDE_MIN_GREEN_MS, applied.sideMinGreenMs);
    TEST_ASSERT_EQUAL_UINT32(TrafficConfig::SIDE_MAX_GREEN_MS, applied.sideMaxGreenMs);
    TEST_ASSERT_EQUAL_UINT32(TrafficConfig::SIDE_CLEAR_GAP_MS, applied.sideClearGapMs);
    TEST_ASSERT_EQUAL_UINT32(TrafficConfig::SIDE_YELLOW_MS,    applied.sideYellowMs);
    TEST_ASSERT_EQUAL_UINT32(TrafficConfig::PED_WALK_MS,       applied.pedWalkMs);
}

// 13. Test Disconnected, unvalidated, invalid adapter frame, and stuck-sensor input handling
void test_disconnected_and_invalid_sensor_input(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.update(1000); // -> ALL_RED
    ctrl.update(2000); // -> MAIN_GREEN

    TEST_ASSERT_FALSE(ctrl.isPhysicalIrValidated());
    ctrl.processPhysicalInputs(2100, true, false);
    ctrl.processPhysicalInputs(2500, true, false);
    TEST_ASSERT_FALSE(ctrl.hasSideVehicleDemand());

    ctrl.setPhysicalIrValidated(true);
    HardwareInputs invalidFrame = {false, false, false};
    ctrl.processHardwareAdapter(2600, invalidFrame);
    ctrl.processHardwareAdapter(2800, invalidFrame);
    TEST_ASSERT_FALSE(ctrl.hasSideVehicleDemand());
    TEST_ASSERT_FALSE(ctrl.hasPedestrianDemand());

    ctrl.processPhysicalInputs(3000, true, false);
    ctrl.processPhysicalInputs(3200, true, false);
    TEST_ASSERT_TRUE(ctrl.hasSideVehicleDemand());
    TEST_ASSERT_FALSE(ctrl.isHwIrFaultSuspected());

    ctrl.processPhysicalInputs(33300, true, false);
    TEST_ASSERT_TRUE(ctrl.isHwIrFaultSuspected());
    TEST_ASSERT_FALSE(ctrl.hasSideVehicleDemand());

    ctrl.processPhysicalInputs(34000, true, true);
    ctrl.processPhysicalInputs(34200, true, true);
    TEST_ASSERT_FALSE(ctrl.isHwIrFaultSuspected());
}

// ============================================================================
// PART II: EXTENDED COMPREHENSIVE TESTS (Tests 14 .. 31 covering all 18 areas)
// ============================================================================

// 14. Test SignalOutputDriver polarity & 5V Common-Ground vehicle safeguard
void test_signal_output_driver_polarity_and_common_ground_safeguard(void) {
    SignalOutputDriver drv;
    // By default, vehicle driver is UNVERIFIED_SAFE_DISABLED because 5V common-ground
    // vehicle modules cannot be sourced by low-side ULN2003A ICs.
    TEST_ASSERT_FALSE(drv.isVehicleDriverVerified());
    TEST_ASSERT_EQUAL(TrafficConfig::OutputDriverType::UNVERIFIED_SAFE_DISABLED, drv.getVehicleDriverType());

    LogicalSignalOutputs activeMain = TrafficController::deriveOutputsForState(TrafficState::MAIN_GREEN);
    ElectricalPinStates elec;
    drv.computeElectricalStates(activeMain, elec);
    // While vehicle driver is unverified, physical vehicle channels must NOT be energized
    for (uint8_t i = 0; i < 9; ++i) {
        TEST_ASSERT_FALSE(elec.channelEnergized[i]);
    }

    // Verify Active-LOW high-side sourcing driver sets pin HIGH when logical lamp is OFF
    // (proving the driver abstraction never assumes GPIO LOW == lamp OFF).
    drv.setVehicleDriverConfig(TrafficConfig::OutputDriverType::HIGH_SIDE_SOURCING_ACTIVE_LOW, true);
    drv.computeElectricalStates(activeMain, elec);
    TEST_ASSERT_TRUE(elec.channelEnergized[0]);  // Main Green ON
    TEST_ASSERT_FALSE(elec.pinHigh[0]);          // Active-LOW: pin driven LOW to turn ON
    TEST_ASSERT_FALSE(elec.channelEnergized[1]); // Main Yellow OFF
    TEST_ASSERT_TRUE(elec.pinHigh[1]);           // Active-LOW: pin held HIGH to keep OFF!

    // Verify Pedestrian ULN2003A low-side driver (Active-HIGH input sinks current for common-anode LEDs)
    drv.setPedestrianDriverConfig(TrafficConfig::OutputDriverType::LOW_SIDE_ULN2003A_ACTIVE_HIGH, true);
    drv.computeElectricalStates(activeMain, elec);
    TEST_ASSERT_FALSE(elec.channelEnergized[9]); // Ped1 Walk OFF
    TEST_ASSERT_FALSE(elec.pinHigh[9]);          // LOW = ULN2003A OFF
    TEST_ASSERT_TRUE(elec.channelEnergized[10]); // Ped1 Don't Walk ON
    TEST_ASSERT_TRUE(elec.pinHigh[10]);          // HIGH = ULN2003A sinks current
}

// 15. Test Every permitted phase transition (including commissioned Protected Turn)
void test_every_permitted_phase_transition_including_turn(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.setSimulationMode(0, true);
    ctrl.setProtectedTurnCommissioned(0, true);

    ctrl.update(1000); // STARTUP_SAFE -> ALL_RED
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());

    ctrl.update(2000); // ALL_RED -> MAIN_GREEN
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());

    // Request Protected Turn demand
    TEST_ASSERT_TRUE(ctrl.setSimMovementDemand(2500, TrafficConfig::MovementId::PROTECTED_TURN, true));
    ctrl.update(6000); // MAIN_GREEN -> MAIN_YELLOW
    TEST_ASSERT_EQUAL(TrafficState::MAIN_YELLOW, ctrl.getState());

    ctrl.update(8000); // MAIN_YELLOW -> ALL_RED
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());

    ctrl.update(9000); // ALL_RED -> TURN_GREEN
    TEST_ASSERT_EQUAL(TrafficState::TURN_GREEN, ctrl.getState());
    SignalOutputs turnOut = ctrl.getOutputs();
    TEST_ASSERT_TRUE(turnOut.turnGreen);
    TEST_ASSERT_TRUE(turnOut.mainRed);
    TEST_ASSERT_TRUE(turnOut.sideRed);
    TEST_ASSERT_TRUE(turnOut.pedDontWalk);
    TEST_ASSERT_TRUE(turnOut.ped2DontWalk);

    // Clear turn demand at 9500 ms -> min green (2000 ms) + gap (2000 ms) -> TURN_YELLOW at 11500 ms
    ctrl.setSimMovementDemand(9500, TrafficConfig::MovementId::PROTECTED_TURN, false);
    ctrl.update(11500);
    TEST_ASSERT_EQUAL(TrafficState::TURN_YELLOW, ctrl.getState());

    ctrl.update(13500); // TURN_YELLOW -> ALL_RED
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());

    ctrl.update(14500); // ALL_RED -> MAIN_GREEN
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());
}

// 16. Test Fixed-Time Operation (`ControlMode::FIXED`)
void test_fixed_time_operation_cycle(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    TEST_ASSERT_TRUE(ctrl.setControlMode(0, ControlMode::FIXED));
    ctrl.setProtectedTurnCommissioned(0, true);

    ctrl.update(1000); // -> ALL_RED
    ctrl.update(2000); // -> MAIN_GREEN (even with zero vehicle demand, FIXED mode cycles!)
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());

    // After mainMinGreenMs (4000 ms) at 6000 ms -> MAIN_YELLOW
    ctrl.update(6000);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_YELLOW, ctrl.getState());
    ctrl.update(8000); // -> ALL_RED
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());
    ctrl.update(9000); // -> SIDE_GREEN
    TEST_ASSERT_EQUAL(TrafficState::SIDE_GREEN, ctrl.getState());

    // After sideMinGreenMs (2000 ms) at 11000 ms -> SIDE_YELLOW
    ctrl.update(11000);
    TEST_ASSERT_EQUAL(TrafficState::SIDE_YELLOW, ctrl.getState());
    ctrl.update(13000); // -> ALL_RED
    ctrl.update(14000); // -> TURN_GREEN (since commissioned)
    TEST_ASSERT_EQUAL(TrafficState::TURN_GREEN, ctrl.getState());

    // After turnMinGreenMs (2000 ms) at 16000 ms -> TURN_YELLOW -> ALL_RED -> MAIN_GREEN
    ctrl.update(16000);
    TEST_ASSERT_EQUAL(TrafficState::TURN_YELLOW, ctrl.getState());
    ctrl.update(18000);
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());
    ctrl.update(19000);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());
}

// 17. Test Vehicle-Actuated Operation across 6 independent IR sensors (`ControlMode::ACTUATED`)
void test_vehicle_actuated_six_ir_sensors(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.setControlMode(0, ControlMode::ACTUATED);
    ctrl.setPhysicalIrValidated(true);
    ctrl.setProtectedTurnCommissioned(0, true);

    ctrl.update(1000); // -> ALL_RED
    ctrl.update(2000); // -> MAIN_GREEN

    // Trigger IR5 (index 4, assigned to PROTECTED_TURN) and IR3 (index 2, assigned to MAIN_ROAD)
    bool irHigh[6] = { true, true, false, true, false, true }; // IR3=LOW (Main), IR5=LOW (Turn)
    ctrl.processAllPhysicalInputs(2100, irHigh, true, true);
    ctrl.processAllPhysicalInputs(2250, irHigh, true, true); // > 100 ms stability
    TEST_ASSERT_TRUE(ctrl.hasMainVehicleDemand());
    TEST_ASSERT_TRUE(ctrl.hasTurnVehicleDemand());
    TEST_ASSERT_FALSE(ctrl.hasSideVehicleDemand());

    // At 6000 ms (mainMinGreenMs = 4000 ms), Main Road still has active presence on IR3,
    // so ACTUATED mode extends MAIN_GREEN up to mainMaxGreenMs (12000 ms -> t=14000 ms)!
    ctrl.update(6000);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());

    // Clear Main Road sensor IR3 at 7000 ms -> MAIN_GREEN yields to waiting TURN_GREEN!
    irHigh[2] = true;
    ctrl.processAllPhysicalInputs(7000, irHigh, true, true);
    ctrl.processAllPhysicalInputs(7150, irHigh, true, true);
    ctrl.update(7200);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_YELLOW, ctrl.getState());

    ctrl.update(9200);  // -> ALL_RED
    ctrl.update(10200); // -> TURN_GREEN (skips SIDE_GREEN because Side has zero demand!)
    TEST_ASSERT_EQUAL(TrafficState::TURN_GREEN, ctrl.getState());
}

// 18. Test Adaptive Demand Selection & Scoring (`ControlMode::ADAPTIVE`)
void test_adaptive_demand_selection_and_scoring(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.setControlMode(0, ControlMode::ADAPTIVE);
    ctrl.setPhysicalIrValidated(true);
    ctrl.setProtectedTurnCommissioned(0, true);

    ctrl.update(1000); // -> ALL_RED
    ctrl.update(2000); // -> MAIN_GREEN

    // Side Road has 1 sensor active (IR1), while Protected Turn has 2 sensors active (IR5 + IR6 queue)
    bool irHigh[6] = { false, true, true, true, false, false }; // IR1=LOW, IR5=LOW, IR6=LOW
    ctrl.processAllPhysicalInputs(2100, irHigh, true, true);
    ctrl.processAllPhysicalInputs(2250, irHigh, true, true);

    const uint32_t sideScore = ctrl.getDemandManager().calculateScore(TrafficConfig::MovementId::SIDE_ROAD);
    const uint32_t turnScore = ctrl.getDemandManager().calculateScore(TrafficConfig::MovementId::PROTECTED_TURN);
    TEST_ASSERT_TRUE(turnScore > sideScore);

    // At 6000 ms, ADAPTIVE mode selects PROTECTED_TURN first because its demand score is higher
    ctrl.update(6000); // -> MAIN_YELLOW
    ctrl.update(8000); // -> ALL_RED
    ctrl.update(9000); // -> TURN_GREEN
    TEST_ASSERT_EQUAL(TrafficState::TURN_GREEN, ctrl.getState());
}

// 19. Test Waiting-Time Fairness & Anti-Starvation in Adaptive Mode
void test_waiting_time_fairness_and_starvation_prevention(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.setControlMode(0, ControlMode::ADAPTIVE);
    ctrl.setPhysicalIrValidated(true);
    ctrl.setProtectedTurnCommissioned(0, true);

    ctrl.update(1000); // -> ALL_RED
    ctrl.update(2000); // -> MAIN_GREEN

    // Side Road has 1 sensor (IR1) waiting since t=2200, Turn has 2 sensors (IR5+IR6)
    bool irHigh[6] = { false, true, true, true, false, false };
    ctrl.processAllPhysicalInputs(2100, irHigh, true, true);
    ctrl.processAllPhysicalInputs(2250, irHigh, true, true);

    ctrl.update(6000); // -> MAIN_YELLOW (Turn chosen first; Side skippedCycles becomes 1, then 2)
    ctrl.update(8000); // -> ALL_RED
    ctrl.update(9000); // -> TURN_GREEN

    // Turn reaches turnMaxGreenMs (6000 ms) at 15000 ms -> TURN_YELLOW -> ALL_RED
    ctrl.update(15000);
    TEST_ASSERT_EQUAL(TrafficState::TURN_YELLOW, ctrl.getState());
    ctrl.update(17000);
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());

    // Now Side Road has waited > 14 seconds AND has been skipped twice (starvation boost active!),
    // so Side Road's score exceeds Turn's score and Side Road is served next!
    const uint32_t sideScoreAfterWait = ctrl.getDemandManager().calculateScore(TrafficConfig::MovementId::SIDE_ROAD);
    const uint32_t turnScoreAfterService = ctrl.getDemandManager().calculateScore(TrafficConfig::MovementId::PROTECTED_TURN);
    TEST_ASSERT_TRUE(sideScoreAfterWait > turnScoreAfterService);

    ctrl.update(18000);
    TEST_ASSERT_EQUAL(TrafficState::SIDE_GREEN, ctrl.getState());
}

// 20. Test Main Road and Protected Turn Maximum Green Enforcement
void test_main_and_turn_max_green_enforcement(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.setControlMode(0, ControlMode::ACTUATED);
    ctrl.setSimulationMode(0, true);
    ctrl.setProtectedTurnCommissioned(0, true);

    ctrl.update(1000); // -> ALL_RED
    ctrl.update(2000); // -> MAIN_GREEN (starts at 2000 ms)

    // Continuous demand on both MAIN_ROAD and PROTECTED_TURN
    ctrl.setSimMovementDemand(2100, TrafficConfig::MovementId::MAIN_ROAD, true);
    ctrl.setSimMovementDemand(2100, TrafficConfig::MovementId::PROTECTED_TURN, true);

    // Main Green extends past minGreen (6000 ms) but MUST terminate at mainMaxGreenMs (12000 ms -> t=14000 ms)
    ctrl.update(13999);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());
    ctrl.update(14000);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_YELLOW, ctrl.getState());

    ctrl.update(16000); // -> ALL_RED
    ctrl.update(17000); // -> TURN_GREEN (starts at 17000 ms)
    TEST_ASSERT_EQUAL(TrafficState::TURN_GREEN, ctrl.getState());

    // Turn Green MUST terminate at turnMaxGreenMs (6000 ms -> t=23000 ms) despite continuous Turn demand
    ctrl.update(22999);
    TEST_ASSERT_EQUAL(TrafficState::TURN_GREEN, ctrl.getState());
    ctrl.update(23000);
    TEST_ASSERT_EQUAL(TrafficState::TURN_YELLOW, ctrl.getState());
}

// 21. Test 6-Sensor Debounce & Disconnected Chatter Filtering
void test_six_ir_sensor_debounce_and_chatter_filtering(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.setPhysicalIrValidated(true);
    ctrl.update(1000);
    ctrl.update(2000);

    // Rapid oscillation (> 12 transitions within 1000 ms) on IR1 simulates a floating/chattering wire
    bool irHigh[6] = { true, true, true, true, true, true };
    for (uint8_t k = 0; k < 16; ++k) {
        irHigh[0] = (k % 2 == 0);
        ctrl.processAllPhysicalInputs(2100 + k * 10, irHigh, true, true);
    }

    TEST_ASSERT_TRUE(ctrl.getVehicleDetector().getChannel(0).chatterFault);
    TEST_ASSERT_FALSE(ctrl.hasSideVehicleDemand());
    TEST_ASSERT_TRUE((ctrl.getFaultManager().getActiveFaultMask() &
                      static_cast<uint16_t>(FaultCode::SENSOR_DISCONNECTED_CHATTER)) != 0);
}

// 22. Test Multi-Sensor Stuck-Active Isolation (Fault on IR1 does not blind IR2)
void test_multi_sensor_stuck_active_isolation(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.setPhysicalIrValidated(true);
    ctrl.update(1000);
    ctrl.update(2000);

    // IR1 (index 0, Side Primary) gets stuck LOW for > 30000 ms
    bool irHigh[6] = { false, true, true, true, true, true };
    ctrl.processAllPhysicalInputs(3000, irHigh, true, true);
    ctrl.processAllPhysicalInputs(3200, irHigh, true, true);
    TEST_ASSERT_TRUE(ctrl.hasSideVehicleDemand());

    ctrl.processAllPhysicalInputs(33300, irHigh, true, true);
    TEST_ASSERT_TRUE(ctrl.getVehicleDetector().getChannel(0).stuckFault);
    TEST_ASSERT_FALSE(ctrl.hasSideVehicleDemand()); // Stuck IR1 suppressed

    // Now a vehicle arrives on IR2 (index 1, Side Secondary) while IR1 is still stuck!
    irHigh[1] = false;
    ctrl.processAllPhysicalInputs(34000, irHigh, true, true);
    ctrl.processAllPhysicalInputs(34200, irHigh, true, true);
    TEST_ASSERT_TRUE(ctrl.hasSideVehicleDemand()); // Healthy IR2 still detects Side demand!
}

// 23. Test Dual Independent Pedestrian Buttons & Signal Groups (PED1 & PED2)
void test_dual_pedestrian_request_queueing(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.update(1000); // -> ALL_RED
    ctrl.update(2000); // -> MAIN_GREEN

    // Press only PED 2 button (GPIO 4) for >= 50 ms
    bool irIdle[6] = { true, true, true, true, true, true };
    ctrl.processAllPhysicalInputs(2500, irIdle, true, false); // PED1=HIGH, PED2=LOW
    ctrl.processAllPhysicalInputs(2560, irIdle, true, false);

    TEST_ASSERT_FALSE(ctrl.hasPedestrianChannelDemand(0));
    TEST_ASSERT_TRUE(ctrl.hasPedestrianChannelDemand(1));
    TEST_ASSERT_TRUE(ctrl.hasPedestrianDemand());

    ctrl.update(6000); // -> MAIN_YELLOW
    ctrl.update(8000); // -> ALL_RED
    ctrl.update(9000); // -> PED_WALK
    TEST_ASSERT_EQUAL(TrafficState::PED_WALK, ctrl.getState());

    SignalOutputs out = ctrl.getOutputs();
    // Only PED2 was requested, so PED2 shows WALK and PED1 stays DONT_WALK!
    TEST_ASSERT_FALSE(out.pedWalk);
    TEST_ASSERT_TRUE(out.pedDontWalk);
    TEST_ASSERT_TRUE(out.ped2Walk);
    TEST_ASSERT_FALSE(out.ped2DontWalk);
    TEST_ASSERT_TRUE(out.mainRed && out.sideRed && out.turnRed);
}

// 24. Test Pedestrian WALK and Configurable Pedestrian Clearance Interval
void test_pedestrian_walk_and_configurable_clearance(void) {
    TrafficController ctrl;
    ctrl.begin(0);

    TrafficConfig::TimingConfig cfg = TrafficConfig::DEFAULT_TIMING_CONFIG;
    cfg.pedClearanceMs = 3000; // Configure 3-second post-WALK pedestrian clearance
    TEST_ASSERT_TRUE(ctrl.setTimingConfig(0, cfg));

    ctrl.setSimulationMode(0, true);
    ctrl.update(1000); // -> ALL_RED
    ctrl.update(2000); // -> MAIN_GREEN
    ctrl.triggerSimPedestrianChannel(2500, 0);
    ctrl.triggerSimPedestrianChannel(2500, 1);

    ctrl.update(6000); // -> MAIN_YELLOW
    ctrl.update(8000); // -> ALL_RED
    ctrl.update(9000); // -> PED_WALK (both PED1 and PED2 walking)
    SignalOutputs walkOut = ctrl.getOutputs();
    TEST_ASSERT_TRUE(walkOut.pedWalk && walkOut.ped2Walk);
    TEST_ASSERT_FALSE(walkOut.pedDontWalk || walkOut.ped2DontWalk);

    // At 14000 ms (5000 ms walk complete) -> enters ALL_RED with pedestrian clearance active
    ctrl.update(14000);
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());
    TEST_ASSERT_TRUE(ctrl.isPedestrianClearanceActive());
    SignalOutputs clrOut = ctrl.getOutputs();
    TEST_ASSERT_FALSE(clrOut.pedWalk || clrOut.ped2Walk);
    TEST_ASSERT_TRUE(clrOut.pedDontWalk && clrOut.ped2DontWalk);
    TEST_ASSERT_TRUE(clrOut.mainRed && clrOut.sideRed && clrOut.turnRed);

    // At 15500 ms (1500 ms elapsed < 3000 ms pedClearanceMs), still in clearance!
    ctrl.update(15500);
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());
    TEST_ASSERT_TRUE(ctrl.isPedestrianClearanceActive());

    // At 17000 ms (full 3000 ms clearance elapsed) -> MAIN_GREEN
    ctrl.update(17000);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());
    TEST_ASSERT_FALSE(ctrl.isPedestrianClearanceActive());
}

// 25. Test Protected-Turn Conflict Prevention & Uncommissioned Gate (Keep-Right)
void test_protected_turn_conflict_prevention_and_commissioning_gate(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.setSimulationMode(0, true);

    // By default, Protected Turn is UNCOMMISSIONED (false) because physical junction
    // geometry has not been confirmed.
    TEST_ASSERT_FALSE(ctrl.isProtectedTurnCommissioned());

    ctrl.update(1000); // -> ALL_RED
    ctrl.update(2000); // -> MAIN_GREEN

    // Even if simulated Turn demand is injected, uncommissioned Turn must NOT be served!
    ctrl.setSimMovementDemand(2500, TrafficConfig::MovementId::PROTECTED_TURN, true);
    TEST_ASSERT_FALSE(ctrl.hasTurnVehicleDemand());
    ctrl.update(6000);
    ctrl.update(10000);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());

    // Attempting to apply a Left-Hand Traffic or conflict-disabled matrix must be rejected
    TrafficConfig::ConflictMatrixConfig badMatrix = TrafficConfig::DEFAULT_CONFLICT_MATRIX;
    badMatrix.rightHandTrafficRule = false;
    badMatrix.conflicts[0][2]      = false; // Illegal: claims Main and Turn don't conflict
    TEST_ASSERT_FALSE(ctrl.setConflictMatrixConfig(10500, badMatrix));
    TEST_ASSERT_TRUE(ctrl.getConflictMatrixConfig().rightHandTrafficRule);
    TEST_ASSERT_TRUE(ctrl.getConflictMatrixConfig().conflicts[0][2]);
}

// 26. Test Manual Mode Safety (`ControlMode::MANUAL` never bypasses Yellow or All-Red)
void test_manual_mode_safety_transitions(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.update(1000); // -> ALL_RED
    ctrl.update(2000); // -> MAIN_GREEN

    TEST_ASSERT_TRUE(ctrl.setControlMode(2000, ControlMode::MANUAL));

    // Without NEXT command, controller stays in MAIN_GREEN indefinitely
    ctrl.update(25000);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());

    // Operator sends NEXT at 25500 ms -> transitions to MAIN_YELLOW (never jumps straight to SIDE_GREEN!)
    TEST_ASSERT_TRUE(ctrl.advanceManualStep(25500));
    ctrl.update(25500);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_YELLOW, ctrl.getState());

    // Must complete full MAIN_YELLOW (2000 ms) and ALL_RED (1000 ms) before SIDE_GREEN
    ctrl.update(27499);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_YELLOW, ctrl.getState());
    ctrl.update(27500);
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());
    ctrl.update(28499);
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());
    ctrl.update(28500);
    TEST_ASSERT_EQUAL(TrafficState::SIDE_GREEN, ctrl.getState());
}

// 27. Test Fault Entry & Explicit Recovery Procedure
void test_fault_entry_and_explicit_recovery(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.update(1000);
    ctrl.update(2000);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());

    ctrl.triggerFault(3000, "TEST_CRITICAL_FAULT", FaultCode::ILLEGAL_SIGNAL_COMBINATION);
    TEST_ASSERT_EQUAL(TrafficState::FAULT_SAFE, ctrl.getState());
    TEST_ASSERT_TRUE(ctrl.getFaultManager().isCriticalFaultLatched());

    // Mode changes and manual steps must be rejected while in FAULT_SAFE
    TEST_ASSERT_FALSE(ctrl.setControlMode(3500, ControlMode::FIXED));
    TEST_ASSERT_FALSE(ctrl.advanceManualStep(3500));

    // Explicit recovery via clearFaultAndRecover must enter STARTUP_SAFE -> ALL_RED -> MAIN_GREEN
    // (never immediately introducing a conflicting green)
    TEST_ASSERT_TRUE(ctrl.clearFaultAndRecover(4000, "UNIT_TEST_CLEAR"));
    TEST_ASSERT_FALSE(ctrl.getFaultManager().isCriticalFaultLatched());
    TEST_ASSERT_EQUAL(TrafficState::STARTUP_SAFE, ctrl.getState());

    ctrl.update(5000);
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());
    ctrl.update(6000);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());
}

// 28. Test Invalid Sensor & Demand Configuration Handling
void test_invalid_sensor_and_demand_config_rejection(void) {
    TrafficController ctrl;
    ctrl.begin(0);

    TrafficConfig::SensorConfig badSens = TrafficConfig::DEFAULT_SENSOR_CONFIG;
    badSens.irStabilityMs       = 0;   // Invalid
    badSens.pedDebounceMs       = 2;   // Invalid
    badSens.maxInputStuckMs     = 100; // Invalid
    badSens.sensorToMovement[0] = 99;  // Invalid movement ID
    TEST_ASSERT_FALSE(ctrl.setSensorConfig(100, badSens));
    TEST_ASSERT_EQUAL_UINT32(TrafficConfig::IR_STABILITY_MS, ctrl.getSensorConfig().irStabilityMs);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(TrafficConfig::MovementId::SIDE_ROAD),
                            ctrl.getSensorConfig().sensorToMovement[0]);

    TrafficConfig::DemandConfig badDem = { 0, 0, 0, 0, 0 };
    TEST_ASSERT_FALSE(ctrl.setDemandConfig(200, badDem));
    TEST_ASSERT_EQUAL_UINT32(TrafficConfig::DEFAULT_WAIT_TIME_WEIGHT_MS, ctrl.getDemandConfig().waitTimeWeightMs);
}

// 29. Test NVS Persistence (Save, Load, and Corrupted Checksum Fallback)
void test_nvs_persistence_save_load_and_corruption_fallback(void) {
    TrafficController ctrl;
    ctrl.getStorage().setUseHardwareNvs(false); // Use deterministic shadow NVS in test
    ctrl.begin(0);

    ctrl.setControlMode(0, ControlMode::ADAPTIVE);
    TrafficConfig::TimingConfig customTimings = TrafficConfig::DEFAULT_TIMING_CONFIG;
    customTimings.mainMinGreenMs = 6500;
    customTimings.pedClearanceMs = 2500;
    TEST_ASSERT_TRUE(ctrl.setTimingConfig(0, customTimings));
    TEST_ASSERT_TRUE(ctrl.saveConfigToNvs(100));

    // Modify controller settings in RAM, then load back saved configuration from NVS
    ctrl.setControlMode(200, ControlMode::FIXED);
    ctrl.setTimingConfig(200, TrafficConfig::DEFAULT_TIMING_CONFIG);
    TEST_ASSERT_EQUAL(ControlMode::FIXED, ctrl.getControlMode());

    TEST_ASSERT_TRUE(ctrl.loadConfigFromNvs(300));
    TEST_ASSERT_EQUAL(ControlMode::ADAPTIVE, ctrl.getControlMode());
    TEST_ASSERT_EQUAL_UINT32(6500, ctrl.getTimingConfig().mainMinGreenMs);
    TEST_ASSERT_EQUAL_UINT32(2500, ctrl.getTimingConfig().pedClearanceMs);

    // Corrupt the stored NVS blob checksum and verify safe fallback to defaults
    PersistentControllerConfig corrupted = ctrl.exportPersistentConfig();
    corrupted.timings.mainMinGreenMs = 999999; // Out of bounds
    corrupted.checksum ^= 0xDEADBEEFUL;        // Bad checksum
    ctrl.getStorage().injectRawShadowBlobForTest(corrupted);

    TEST_ASSERT_FALSE(ctrl.loadConfigFromNvs(400));
    TEST_ASSERT_EQUAL(ControlMode::AUTO, ctrl.getControlMode());
    TEST_ASSERT_EQUAL_UINT32(TrafficConfig::MAIN_MIN_GREEN_MS, ctrl.getTimingConfig().mainMinGreenMs);
}

// 30. Test Serial Command Interface & Invalid Command Rejection
void test_serial_command_interface_and_invalid_commands(void) {
    TrafficController ctrl;
    ctrl.getStorage().setUseHardwareNvs(false);
    ctrl.begin(0);
    ctrl.update(1000); // -> ALL_RED
    ctrl.update(2000); // -> MAIN_GREEN

    SerialCommandHandler cmdHandler;

    // Valid mode commands
    TEST_ASSERT_EQUAL(SerialCommandResult::OK_EXECUTED, cmdHandler.executeCommand(2100, "MODE FIXED", ctrl));
    TEST_ASSERT_EQUAL(ControlMode::FIXED, ctrl.getControlMode());

    TEST_ASSERT_EQUAL(SerialCommandResult::OK_EXECUTED, cmdHandler.executeCommand(2200, "MODE ACTUATED", ctrl));
    TEST_ASSERT_EQUAL(ControlMode::ACTUATED, ctrl.getControlMode());

    TEST_ASSERT_EQUAL(SerialCommandResult::OK_EXECUTED, cmdHandler.executeCommand(2300, "MODE ADAPTIVE", ctrl));
    TEST_ASSERT_EQUAL(ControlMode::ADAPTIVE, ctrl.getControlMode());

    TEST_ASSERT_EQUAL(SerialCommandResult::OK_EXECUTED, cmdHandler.executeCommand(2400, "MODE AUTO", ctrl));
    TEST_ASSERT_EQUAL(ControlMode::AUTO, ctrl.getControlMode());

    // Pedestrian & Vehicle commands
    TEST_ASSERT_EQUAL(SerialCommandResult::OK_EXECUTED, cmdHandler.executeCommand(2500, "PED 1", ctrl));
    TEST_ASSERT_TRUE(ctrl.hasPedestrianChannelDemand(0));
    TEST_ASSERT_EQUAL(SerialCommandResult::OK_EXECUTED, cmdHandler.executeCommand(2550, "PED 2", ctrl));
    TEST_ASSERT_TRUE(ctrl.hasPedestrianChannelDemand(1));
    TEST_ASSERT_EQUAL(SerialCommandResult::OK_EXECUTED, cmdHandler.executeCommand(2600, "CAR ON", ctrl));
    TEST_ASSERT_TRUE(ctrl.hasSideVehicleDemand());
    TEST_ASSERT_EQUAL(SerialCommandResult::OK_EXECUTED, cmdHandler.executeCommand(2650, "CAR OFF", ctrl));
    TEST_ASSERT_FALSE(ctrl.hasSideVehicleDemand());

    // Read-only telemetry commands
    TEST_ASSERT_EQUAL(SerialCommandResult::OK_EXECUTED, cmdHandler.executeCommand(2700, "STATUS", ctrl));
    TEST_ASSERT_EQUAL(SerialCommandResult::OK_EXECUTED, cmdHandler.executeCommand(2700, "SENSORS", ctrl));
    TEST_ASSERT_EQUAL(SerialCommandResult::OK_EXECUTED, cmdHandler.executeCommand(2700, "DEMAND", ctrl));
    TEST_ASSERT_EQUAL(SerialCommandResult::OK_EXECUTED, cmdHandler.executeCommand(2700, "PHASES", ctrl));
    TEST_ASSERT_EQUAL(SerialCommandResult::OK_EXECUTED, cmdHandler.executeCommand(2700, "TIMING", ctrl));
    TEST_ASSERT_EQUAL(SerialCommandResult::OK_EXECUTED, cmdHandler.executeCommand(2700, "CONFIG", ctrl));

    // Invalid commands must return UNKNOWN_INVALID_CMD and MUST NOT change controller state
    const TrafficState stateBefore = ctrl.getState();
    const ControlMode  modeBefore  = ctrl.getControlMode();
    TEST_ASSERT_EQUAL(SerialCommandResult::UNKNOWN_INVALID_CMD,
                      cmdHandler.executeCommand(2800, "MODE FLYING_CARS", ctrl));
    TEST_ASSERT_EQUAL(SerialCommandResult::UNKNOWN_INVALID_CMD,
                      cmdHandler.executeCommand(2800, "FORCE_GREEN MAIN", ctrl));
    TEST_ASSERT_EQUAL(stateBefore, ctrl.getState());
    TEST_ASSERT_EQUAL(modeBefore, ctrl.getControlMode());

    // FAULT and CLEAR commands
    TEST_ASSERT_EQUAL(SerialCommandResult::OK_EXECUTED, cmdHandler.executeCommand(2900, "FAULT", ctrl));
    TEST_ASSERT_EQUAL(TrafficState::FAULT_SAFE, ctrl.getState());
    TEST_ASSERT_EQUAL(SerialCommandResult::OK_EXECUTED, cmdHandler.executeCommand(3000, "CLEAR", ctrl));
    TEST_ASSERT_EQUAL(TrafficState::STARTUP_SAFE, ctrl.getState());
}

// 31. Test Every Safety Invariant on 13 Channels & Emergency Priority IR Guard
void test_every_safety_invariant_13_channels_and_emergency_guard(void) {
    const char* reason = nullptr;

    // 1. Conflicting Turn Green + Main Green
    SignalOutputs mainAndTurnGreen = TrafficController::deriveOutputsForState(TrafficState::MAIN_GREEN);
    mainAndTurnGreen.turnGreen = true;
    mainAndTurnGreen.turnRed   = false;
    TEST_ASSERT_FALSE(TrafficController::verifySafetyInvariants(TrafficState::MAIN_GREEN, mainAndTurnGreen, &reason));

    // 2. Conflicting Turn Green + Side Green
    SignalOutputs sideAndTurnGreen = TrafficController::deriveOutputsForState(TrafficState::SIDE_GREEN);
    sideAndTurnGreen.turnGreen = true;
    sideAndTurnGreen.turnRed   = false;
    TEST_ASSERT_FALSE(TrafficController::verifySafetyInvariants(TrafficState::SIDE_GREEN, sideAndTurnGreen, &reason));

    // 3. Turn Green + Turn Red simultaneous on Group 3
    SignalOutputs turnRedAndGreen = TrafficController::deriveOutputsForState(TrafficState::TURN_GREEN);
    turnRedAndGreen.turnRed = true;
    TEST_ASSERT_FALSE(TrafficController::verifySafetyInvariants(TrafficState::TURN_GREEN, turnRedAndGreen, &reason));

    // 4. Ped 2 Walk + Ped 2 Don't Walk simultaneous
    SignalOutputs ped2Conflict = TrafficController::deriveOutputsForState(TrafficState::PED_WALK);
    ped2Conflict.ped2Walk     = true;
    ped2Conflict.ped2DontWalk = true;
    TEST_ASSERT_FALSE(TrafficController::verifySafetyInvariants(TrafficState::PED_WALK, ped2Conflict, &reason));

    // 5. Ped 2 Walk active during Turn Green
    SignalOutputs ped2DuringTurn = TrafficController::deriveOutputsForState(TrafficState::TURN_GREEN);
    ped2DuringTurn.ped2Walk     = true;
    ped2DuringTurn.ped2DontWalk = false;
    TEST_ASSERT_FALSE(TrafficController::verifySafetyInvariants(TrafficState::TURN_GREEN, ped2DuringTurn, &reason));

    // 6. Emergency Priority interface must reject IR_SENSOR as a trigger source
    TrafficController ctrl;
    ctrl.begin(0);
    TEST_ASSERT_FALSE(ctrl.requestEmergencyPriority(100, TrafficConfig::MovementId::MAIN_ROAD, "IR_SENSOR"));
    TEST_ASSERT_FALSE(ctrl.isEmergencyPriorityActive());
    // Validated external transponder source is accepted for a commissioned movement
    TEST_ASSERT_TRUE(ctrl.requestEmergencyPriority(200, TrafficConfig::MovementId::SIDE_ROAD, "EXTERNAL_TRANSPONDER"));
    TEST_ASSERT_TRUE(ctrl.isEmergencyPriorityActive());
}

void setup() {
    delay(2000);
    UNITY_BEGIN();
    // Preserved v1.1.0 regression tests (1..13)
    RUN_TEST(test_startup_to_all_red_to_main_green);
    RUN_TEST(test_side_road_service_and_early_termination);
    RUN_TEST(test_side_road_max_green);
    RUN_TEST(test_pedestrian_service);
    RUN_TEST(test_simultaneous_pedestrian_and_vehicle_requests);
    RUN_TEST(test_simulation_mode_rejection_and_sim_off_cleanup);
    RUN_TEST(test_reset_during_every_normal_phase);
    RUN_TEST(test_millis_rollover_safety);
    RUN_TEST(test_safety_invariants_and_fault_safe);
    RUN_TEST(test_physical_button_debounce_and_hold_suppression);
    RUN_TEST(test_no_demand_continuous_main_green);
    RUN_TEST(test_invalid_timing_configuration_fallback);
    RUN_TEST(test_disconnected_and_invalid_sensor_input);

    // Extended v2.0.0 comprehensive tests (14..31)
    RUN_TEST(test_signal_output_driver_polarity_and_common_ground_safeguard);
    RUN_TEST(test_every_permitted_phase_transition_including_turn);
    RUN_TEST(test_fixed_time_operation_cycle);
    RUN_TEST(test_vehicle_actuated_six_ir_sensors);
    RUN_TEST(test_adaptive_demand_selection_and_scoring);
    RUN_TEST(test_waiting_time_fairness_and_starvation_prevention);
    RUN_TEST(test_main_and_turn_max_green_enforcement);
    RUN_TEST(test_six_ir_sensor_debounce_and_chatter_filtering);
    RUN_TEST(test_multi_sensor_stuck_active_isolation);
    RUN_TEST(test_dual_pedestrian_request_queueing);
    RUN_TEST(test_pedestrian_walk_and_configurable_clearance);
    RUN_TEST(test_protected_turn_conflict_prevention_and_commissioning_gate);
    RUN_TEST(test_manual_mode_safety_transitions);
    RUN_TEST(test_fault_entry_and_explicit_recovery);
    RUN_TEST(test_invalid_sensor_and_demand_config_rejection);
    RUN_TEST(test_nvs_persistence_save_load_and_corruption_fallback);
    RUN_TEST(test_serial_command_interface_and_invalid_commands);
    RUN_TEST(test_every_safety_invariant_13_channels_and_emergency_guard);
    UNITY_END();
}

void loop() {}
