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
    // By default, vehicle driver topology is DIRECT_ACTIVE_HIGH (unverified/disabled by default)
    // and pedestrian driver topology is LOW_SIDE_ULN2003A_ACTIVE_HIGH (unverified/disabled by default).
    TEST_ASSERT_FALSE(drv.isVehicleDriverVerified());
    TEST_ASSERT_FALSE(drv.isPedestrianDriverVerified());
    TEST_ASSERT_EQUAL(TrafficConfig::OutputDriverType::DIRECT_ACTIVE_HIGH, drv.getVehicleDriverType());
    TEST_ASSERT_EQUAL(TrafficConfig::OutputDriverType::LOW_SIDE_ULN2003A_ACTIVE_HIGH, drv.getPedestrianDriverType());

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

    // Trigger IR1 (index 0, GPIO 33, assigned to MAIN_ROAD S1) and IR5 (index 4, GPIO 39, assigned to PROTECTED_TURN S1)
    bool irHigh[6] = { false, true, true, true, false, true }; // IR1=LOW (Main S1), IR5=LOW (Turn S1)
    ctrl.processAllPhysicalInputs(2100, irHigh, true, true);
    ctrl.processAllPhysicalInputs(2250, irHigh, true, true); // > 100 ms stability
    TEST_ASSERT_TRUE(ctrl.hasMainVehicleDemand());
    TEST_ASSERT_TRUE(ctrl.hasTurnVehicleDemand());
    TEST_ASSERT_FALSE(ctrl.hasSideVehicleDemand());

    // At 6000 ms (mainMinGreenMs = 4000 ms), Main Road still has active presence on IR1 (GPIO 33),
    // so ACTUATED mode extends MAIN_GREEN up to mainMaxGreenMs (12000 ms -> t=14000 ms)!
    ctrl.update(6000);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());

    // Clear Main Road sensor IR1 at 7000 ms -> MAIN_GREEN yields to waiting TURN_GREEN!
    irHigh[0] = true;
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
    // Unvalidated IR sensors reject ADAPTIVE mode
    TEST_ASSERT_FALSE(ctrl.setControlMode(0, ControlMode::ADAPTIVE));
    ctrl.setPhysicalIrValidated(true);
    TEST_ASSERT_TRUE(ctrl.setControlMode(0, ControlMode::ADAPTIVE));
    ctrl.setProtectedTurnCommissioned(0, true);

    ctrl.update(1000); // -> ALL_RED
    ctrl.update(2000); // -> MAIN_GREEN

    // Side Road has 1 sensor active (IR3 / GPIO 35), while Protected Turn has 2 sensors active (IR5 + IR6 / GPIO 39 + 15)
    bool irHigh[6] = { true, true, false, true, false, false }; // IR3=LOW (Side S1), IR5=LOW (Turn S1), IR6=LOW (Turn S2)
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
    // Unvalidated IR sensors reject ADAPTIVE mode
    TEST_ASSERT_FALSE(ctrl.setControlMode(0, ControlMode::ADAPTIVE));
    ctrl.setPhysicalIrValidated(true);
    TEST_ASSERT_TRUE(ctrl.setControlMode(0, ControlMode::ADAPTIVE));
    ctrl.setProtectedTurnCommissioned(0, true);

    ctrl.update(1000); // -> ALL_RED
    ctrl.update(2000); // -> MAIN_GREEN

    // Side Road has 1 sensor (IR3 / GPIO 35) waiting since t=2200, Turn has 2 sensors (IR5+IR6 / GPIO 39+15)
    bool irHigh[6] = { true, true, false, true, false, false };
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

    // Rapid oscillation (> 12 transitions within 1000 ms) on IR3 (index 2, GPIO 35, Side Road S1) simulates a floating/chattering wire
    bool irHigh[6] = { true, true, true, true, true, true };
    for (uint8_t k = 0; k < 16; ++k) {
        irHigh[2] = (k % 2 == 0);
        ctrl.processAllPhysicalInputs(2100 + k * 10, irHigh, true, true);
    }

    TEST_ASSERT_TRUE(ctrl.getVehicleDetector().getChannel(2).chatterFault);
    TEST_ASSERT_FALSE(ctrl.hasSideVehicleDemand());
    TEST_ASSERT_TRUE((ctrl.getFaultManager().getActiveFaultMask() &
                      static_cast<uint16_t>(FaultCode::SENSOR_DISCONNECTED_CHATTER)) != 0);
}

// 22. Test Multi-Sensor Stuck-Active Isolation (Fault on Side S1 does not blind Side S2)
void test_multi_sensor_stuck_active_isolation(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.setPhysicalIrValidated(true);
    ctrl.update(1000);
    ctrl.update(2000);

    // IR3 (index 2, GPIO 35, Side Road Sensor 1) gets stuck LOW for > 30000 ms
    bool irHigh[6] = { true, true, false, true, true, true };
    ctrl.processAllPhysicalInputs(3000, irHigh, true, true);
    ctrl.processAllPhysicalInputs(3200, irHigh, true, true);
    TEST_ASSERT_TRUE(ctrl.hasSideVehicleDemand());

    ctrl.processAllPhysicalInputs(33300, irHigh, true, true);
    TEST_ASSERT_TRUE(ctrl.getVehicleDetector().getChannel(2).stuckFault);
    TEST_ASSERT_FALSE(ctrl.hasSideVehicleDemand()); // Stuck IR3 suppressed

    // Now a vehicle arrives on IR4 (index 3, GPIO 36, Side Road Sensor 2) while IR3 is still stuck!
    irHigh[3] = false;
    ctrl.processAllPhysicalInputs(34000, irHigh, true, true);
    ctrl.processAllPhysicalInputs(34200, irHigh, true, true);
    TEST_ASSERT_TRUE(ctrl.hasSideVehicleDemand()); // Healthy IR4 still detects Side demand!
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
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(TrafficConfig::MovementId::MAIN_ROAD),
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

    ctrl.setPhysicalIrValidated(true);
    TEST_ASSERT_TRUE(ctrl.setControlMode(0, ControlMode::ADAPTIVE));
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

    // MODE ADAPTIVE must be rejected while IR sensors are uncommissioned
    TEST_ASSERT_EQUAL(SerialCommandResult::REJECTED_BY_STATE, cmdHandler.executeCommand(2300, "MODE ADAPTIVE", ctrl));
    TEST_ASSERT_NOT_EQUAL(ControlMode::ADAPTIVE, ctrl.getControlMode());

    // Commission all IR sensors, then MODE ADAPTIVE succeeds
    ctrl.setPhysicalIrValidated(true);
    TEST_ASSERT_EQUAL(SerialCommandResult::OK_EXECUTED, cmdHandler.executeCommand(2350, "MODE ADAPTIVE", ctrl));
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

// 30b. Test Serial Timing Adjustment, Atomic Updates, and NVS Persistence
void test_serial_timing_adjustment_atomic_and_nvs_persistence(void) {
    TrafficController ctrl;
    ctrl.getStorage().setUseHardwareNvs(false);
    ctrl.begin(0);

    SerialCommandHandler cmdHandler;

    // 1. Valid TIMING SET with 12 parameters
    // Main: 22000 28000 3500 | Side: 18000 22000 3000 | Turn: 2500 5000 2000 | Ped: 4500 2500 | AllRed: 2200
    const char* validTimingSet = "TIMING SET 22000 28000 3500 18000 22000 3000 2500 5000 2000 4500 2500 2200";
    TEST_ASSERT_EQUAL(SerialCommandResult::OK_EXECUTED,
                      cmdHandler.executeCommand(1000, validTimingSet, ctrl));

    const TrafficConfig::TimingConfig& t1 = ctrl.getTimingConfig();
    TEST_ASSERT_EQUAL_UINT32(22000, t1.mainMinGreenMs);
    TEST_ASSERT_EQUAL_UINT32(28000, t1.mainMaxGreenMs);
    TEST_ASSERT_EQUAL_UINT32(3500,  t1.mainYellowMs);
    TEST_ASSERT_EQUAL_UINT32(18000, t1.sideMinGreenMs);
    TEST_ASSERT_EQUAL_UINT32(22000, t1.sideMaxGreenMs);
    TEST_ASSERT_EQUAL_UINT32(3000,  t1.sideYellowMs);
    TEST_ASSERT_EQUAL_UINT32(2500,  t1.turnMinGreenMs);
    TEST_ASSERT_EQUAL_UINT32(5000,  t1.turnMaxGreenMs);
    TEST_ASSERT_EQUAL_UINT32(2000,  t1.turnYellowMs);
    TEST_ASSERT_EQUAL_UINT32(4500,  t1.pedWalkMs);
    TEST_ASSERT_EQUAL_UINT32(2500,  t1.pedClearanceMs);
    TEST_ASSERT_EQUAL_UINT32(2200,  t1.allRedMs);

    // Verify NVS persistence: load into a fresh controller
    TrafficController freshCtrl;
    freshCtrl.getStorage().setUseHardwareNvs(false);
    freshCtrl.getStorage().injectRawShadowBlobForTest(ctrl.exportPersistentConfig());
    TEST_ASSERT_TRUE(freshCtrl.loadConfigFromNvs(1100));
    TEST_ASSERT_EQUAL_UINT32(22000, freshCtrl.getTimingConfig().mainMinGreenMs);
    TEST_ASSERT_EQUAL_UINT32(28000, freshCtrl.getTimingConfig().mainMaxGreenMs);
    TEST_ASSERT_EQUAL_UINT32(4500,  freshCtrl.getTimingConfig().pedWalkMs);

    // 2. TIMING SET with insufficient arguments (<12) must be rejected
    TEST_ASSERT_EQUAL(SerialCommandResult::UNKNOWN_INVALID_CMD,
                      cmdHandler.executeCommand(1200, "TIMING SET 20000 30000", ctrl));

    // 3. TIMING SET with MaxGreen < MinGreen must be rejected
    TEST_ASSERT_EQUAL(SerialCommandResult::REJECTED_BY_STATE,
                      cmdHandler.executeCommand(1300, "TIMING SET 30000 20000 3000 18000 22000 3000 2500 5000 2000 4500 2500 2200", ctrl));

    // 4. TIMING SET with Yellow < 1000 ms must be rejected
    TEST_ASSERT_EQUAL(SerialCommandResult::REJECTED_BY_STATE,
                      cmdHandler.executeCommand(1400, "TIMING SET 20000 30000 500 18000 22000 3000 2500 5000 2000 4500 2500 2200", ctrl));

    // 5. TIMING SET with All-Red < 1000 ms must be rejected
    TEST_ASSERT_EQUAL(SerialCommandResult::REJECTED_BY_STATE,
                      cmdHandler.executeCommand(1500, "TIMING SET 20000 30000 3000 18000 22000 3000 2500 5000 2000 4500 2500 500", ctrl));

    // 6. Valid TIMING TURN command with 3 arguments
    TEST_ASSERT_EQUAL(SerialCommandResult::OK_EXECUTED,
                      cmdHandler.executeCommand(1600, "TIMING TURN 3000 2500 7000", ctrl));
    TEST_ASSERT_EQUAL_UINT32(3000, ctrl.getTimingConfig().turnMinGreenMs);
    TEST_ASSERT_EQUAL_UINT32(2500, ctrl.getTimingConfig().turnYellowMs);
    TEST_ASSERT_EQUAL_UINT32(7000, ctrl.getTimingConfig().turnMaxGreenMs);

    // 7. TIMING PROTOTYPE restores canonical prototype values and persists
    TEST_ASSERT_EQUAL(SerialCommandResult::OK_EXECUTED,
                      cmdHandler.executeCommand(1700, "TIMING PROTOTYPE", ctrl));
    TEST_ASSERT_EQUAL_UINT32(TrafficConfig::PROTOTYPE_MAIN_MIN_GREEN_MS, ctrl.getTimingConfig().mainMinGreenMs);
    TEST_ASSERT_EQUAL_UINT32(TrafficConfig::PROTOTYPE_SIDE_MIN_GREEN_MS, ctrl.getTimingConfig().sideMinGreenMs);
    TEST_ASSERT_EQUAL_UINT32(TrafficConfig::PROTOTYPE_MAIN_YELLOW_MS, ctrl.getTimingConfig().mainYellowMs);
    TEST_ASSERT_EQUAL_UINT32(TrafficConfig::PROTOTYPE_ALL_RED_MS, ctrl.getTimingConfig().allRedMs);
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

// 32. Test Locked Physical GPIO Assignments across all 9 Vehicle, 4 Pedestrian, 2 Button, and 6 IR Sensor Pins
void test_locked_gpio_assignments_and_all_13_signal_outputs(void) {
    // 1. Vehicle Traffic Light RYG pins
    TEST_ASSERT_EQUAL_UINT8(27, TrafficConfig::PIN_MAIN_RED);
    TEST_ASSERT_EQUAL_UINT8(26, TrafficConfig::PIN_MAIN_YELLOW);
    TEST_ASSERT_EQUAL_UINT8(25, TrafficConfig::PIN_MAIN_GREEN);

    TEST_ASSERT_EQUAL_UINT8(14, TrafficConfig::PIN_SIDE_RED);
    TEST_ASSERT_EQUAL_UINT8(12, TrafficConfig::PIN_SIDE_YELLOW);
    TEST_ASSERT_EQUAL_UINT8(13, TrafficConfig::PIN_SIDE_GREEN);

    TEST_ASSERT_EQUAL_UINT8(22, TrafficConfig::PIN_TURN_RED);
    TEST_ASSERT_EQUAL_UINT8(23, TrafficConfig::PIN_TURN_YELLOW);
    TEST_ASSERT_EQUAL_UINT8(21, TrafficConfig::PIN_TURN_GREEN);

    // 2. Pedestrian signal & button pins
    TEST_ASSERT_EQUAL_UINT8(19, TrafficConfig::PIN_PED1_WALK);
    TEST_ASSERT_EQUAL_UINT8(18, TrafficConfig::PIN_PED1_DONT_WALK);
    TEST_ASSERT_EQUAL_UINT8(16, TrafficConfig::PIN_PED2_WALK);
    TEST_ASSERT_EQUAL_UINT8(17, TrafficConfig::PIN_PED2_DONT_WALK);
    TEST_ASSERT_EQUAL_UINT8(32, TrafficConfig::PIN_PED1_BUTTON);
    TEST_ASSERT_EQUAL_UINT8(4,  TrafficConfig::PIN_PED2_BUTTON);

    // 3. Six IR sensor pins (2 per road)
    TEST_ASSERT_EQUAL_UINT8(33, TrafficConfig::PIN_MAIN_IR_SENSOR_1);
    TEST_ASSERT_EQUAL_UINT8(34, TrafficConfig::PIN_MAIN_IR_SENSOR_2);
    TEST_ASSERT_EQUAL_UINT8(35, TrafficConfig::PIN_SIDE_IR_SENSOR_1);
    TEST_ASSERT_EQUAL_UINT8(36, TrafficConfig::PIN_SIDE_IR_SENSOR_2);
    TEST_ASSERT_EQUAL_UINT8(39, TrafficConfig::PIN_TURN_IR_SENSOR_1);
    TEST_ASSERT_EQUAL_UINT8(15, TrafficConfig::PIN_TURN_IR_SENSOR_2);

    TEST_ASSERT_EQUAL_UINT8(33, TrafficConfig::ROAD_IR_SENSOR_PINS[0][0]);
    TEST_ASSERT_EQUAL_UINT8(34, TrafficConfig::ROAD_IR_SENSOR_PINS[0][1]);
    TEST_ASSERT_EQUAL_UINT8(35, TrafficConfig::ROAD_IR_SENSOR_PINS[1][0]);
    TEST_ASSERT_EQUAL_UINT8(36, TrafficConfig::ROAD_IR_SENSOR_PINS[1][1]);
    TEST_ASSERT_EQUAL_UINT8(39, TrafficConfig::ROAD_IR_SENSOR_PINS[2][0]);
    TEST_ASSERT_EQUAL_UINT8(15, TrafficConfig::ROAD_IR_SENSOR_PINS[2][1]);

    // 4. Verify SignalOutputDriver channel-to-GPIO lookup for all 13 output channels
    SignalOutputDriver drv;
    TEST_ASSERT_EQUAL_INT8(0,  drv.findChannelByGpio(25)); // Main Green
    TEST_ASSERT_EQUAL_INT8(1,  drv.findChannelByGpio(26)); // Main Yellow
    TEST_ASSERT_EQUAL_INT8(2,  drv.findChannelByGpio(27)); // Main Red
    TEST_ASSERT_EQUAL_INT8(3,  drv.findChannelByGpio(13)); // Side Green (GPIO 13)
    TEST_ASSERT_EQUAL_INT8(4,  drv.findChannelByGpio(12)); // Side Yellow (GPIO 12)
    TEST_ASSERT_EQUAL_INT8(5,  drv.findChannelByGpio(14)); // Side Red (GPIO 14)
    TEST_ASSERT_EQUAL_INT8(6,  drv.findChannelByGpio(21)); // Turn Green (GPIO 21)
    TEST_ASSERT_EQUAL_INT8(7,  drv.findChannelByGpio(23)); // Turn Yellow (GPIO 23)
    TEST_ASSERT_EQUAL_INT8(8,  drv.findChannelByGpio(22)); // Turn Red (GPIO 22)
    TEST_ASSERT_EQUAL_INT8(9,  drv.findChannelByGpio(19)); // Ped 1 Walk
    TEST_ASSERT_EQUAL_INT8(10, drv.findChannelByGpio(18)); // Ped 1 Don't Walk
    TEST_ASSERT_EQUAL_INT8(11, drv.findChannelByGpio(16)); // Ped 2 Walk (GPIO 16)
    TEST_ASSERT_EQUAL_INT8(12, drv.findChannelByGpio(17)); // Ped 2 Don't Walk (GPIO 17)
}

// 33. Test ALL_RED Physical Lamp Safety (Red GPIOs 27, 14, 22 ON; Green GPIOs 25, 13, 21 OFF)
void test_all_red_physical_red_and_green_lamp_mapping(void) {
    const LogicalSignalOutputs allRedLogical = TrafficController::deriveOutputsForState(TrafficState::ALL_RED);
    SignalOutputDriver drv;
    drv.setVehicleDriverConfig(TrafficConfig::OutputDriverType::LOW_SIDE_ULN2003A_ACTIVE_HIGH, true);
    drv.setPedestrianDriverConfig(TrafficConfig::OutputDriverType::LOW_SIDE_ULN2003A_ACTIVE_HIGH, true);

    ElectricalPinStates elec;
    drv.computeElectricalStates(allRedLogical, elec);

    // Physical Red GPIOs (27, 14, 22) and Pedestrian Don't Walk GPIOs (18, 17) MUST be logically ON & energized
    TEST_ASSERT_TRUE(elec.isLogicalSignalOnForGpio(27));
    TEST_ASSERT_TRUE(elec.isLogicalSignalOnForGpio(14));
    TEST_ASSERT_TRUE(elec.isLogicalSignalOnForGpio(22));
    TEST_ASSERT_TRUE(elec.isLampEnergizedForGpio(27));
    TEST_ASSERT_TRUE(elec.isLampEnergizedForGpio(14));
    TEST_ASSERT_TRUE(elec.isLampEnergizedForGpio(22));
    TEST_ASSERT_TRUE(elec.isLampEnergizedForGpio(18));
    TEST_ASSERT_TRUE(elec.isLampEnergizedForGpio(17));

    // Physical Green GPIOs (25, 13, 21), Yellow GPIOs (26, 12, 23), and Walk GPIOs (19, 16) MUST be OFF
    TEST_ASSERT_FALSE(elec.isLogicalSignalOnForGpio(25));
    TEST_ASSERT_FALSE(elec.isLogicalSignalOnForGpio(13));
    TEST_ASSERT_FALSE(elec.isLogicalSignalOnForGpio(21));
    TEST_ASSERT_FALSE(elec.isLampEnergizedForGpio(25));
    TEST_ASSERT_FALSE(elec.isLampEnergizedForGpio(13));
    TEST_ASSERT_FALSE(elec.isLampEnergizedForGpio(21));
    TEST_ASSERT_FALSE(elec.isLampEnergizedForGpio(26));
    TEST_ASSERT_FALSE(elec.isLampEnergizedForGpio(12));
    TEST_ASSERT_FALSE(elec.isLampEnergizedForGpio(23));
    TEST_ASSERT_FALSE(elec.isLampEnergizedForGpio(19));
    TEST_ASSERT_FALSE(elec.isLampEnergizedForGpio(16));
}

// 34. Test Two Independent Sensors Per Road Architecture (Main: 33/34, Side: 35/36, Turn: 39/15)
void test_two_independent_sensors_per_road_architecture(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.setPhysicalIrValidated(true);
    ctrl.setProtectedTurnCommissioned(0, true);
    ctrl.update(1000);
    ctrl.update(2000);

    const VehicleDetector& det = ctrl.getVehicleDetector();
    TEST_ASSERT_EQUAL_UINT8(33, det.getRoadSensor(TrafficConfig::MovementId::MAIN_ROAD, 0).gpioPin);
    TEST_ASSERT_EQUAL_UINT8(34, det.getRoadSensor(TrafficConfig::MovementId::MAIN_ROAD, 1).gpioPin);
    TEST_ASSERT_EQUAL_UINT8(35, det.getRoadSensor(TrafficConfig::MovementId::SIDE_ROAD, 0).gpioPin);
    TEST_ASSERT_EQUAL_UINT8(36, det.getRoadSensor(TrafficConfig::MovementId::SIDE_ROAD, 1).gpioPin);
    TEST_ASSERT_EQUAL_UINT8(39, det.getRoadSensor(TrafficConfig::MovementId::PROTECTED_TURN, 0).gpioPin);
    TEST_ASSERT_EQUAL_UINT8(15, det.getRoadSensor(TrafficConfig::MovementId::PROTECTED_TURN, 1).gpioPin);

    // Activate Main S2 (GPIO 34, index 1), Side S1 (GPIO 35, index 2), and Turn S1+S2 (GPIO 39+15, indices 4,5)
    bool irHigh[6] = { true, false, false, true, false, false };
    ctrl.processAllPhysicalInputs(2100, irHigh, true, true);
    ctrl.processAllPhysicalInputs(2250, irHigh, true, true);

    TEST_ASSERT_FALSE(det.isRoadSensorDetected(TrafficConfig::MovementId::MAIN_ROAD, 0));
    TEST_ASSERT_TRUE(det.isRoadSensorDetected(TrafficConfig::MovementId::MAIN_ROAD, 1));
    TEST_ASSERT_TRUE(det.isRoadSensorDetected(TrafficConfig::MovementId::SIDE_ROAD, 0));
    TEST_ASSERT_FALSE(det.isRoadSensorDetected(TrafficConfig::MovementId::SIDE_ROAD, 1));
    TEST_ASSERT_TRUE(det.isRoadSensorDetected(TrafficConfig::MovementId::PROTECTED_TURN, 0));
    TEST_ASSERT_TRUE(det.isRoadSensorDetected(TrafficConfig::MovementId::PROTECTED_TURN, 1));

    const ApproachDemandState& mainDem = ctrl.getDemandManager().getApproachState(TrafficConfig::MovementId::MAIN_ROAD);
    const ApproachDemandState& sideDem = ctrl.getDemandManager().getApproachState(TrafficConfig::MovementId::SIDE_ROAD);
    const ApproachDemandState& turnDem = ctrl.getDemandManager().getApproachState(TrafficConfig::MovementId::PROTECTED_TURN);

    TEST_ASSERT_FALSE(mainDem.sensor1Active);
    TEST_ASSERT_TRUE(mainDem.sensor2Active);
    TEST_ASSERT_EQUAL_UINT8(1, mainDem.activeSensorCount);

    TEST_ASSERT_TRUE(sideDem.sensor1Active);
    TEST_ASSERT_FALSE(sideDem.sensor2Active);
    TEST_ASSERT_EQUAL_UINT8(1, sideDem.activeSensorCount);

    TEST_ASSERT_TRUE(turnDem.sensor1Active);
    TEST_ASSERT_TRUE(turnDem.sensor2Active);
    TEST_ASSERT_EQUAL_UINT8(2, turnDem.activeSensorCount);
}

// 35. Test 4-Layer SignalOutputDriver Abstraction (Logical vs. GPIO vs. ULN2003A Sink vs. Physical Lamp)
void test_four_layer_signal_output_driver_abstraction(void) {
    SignalOutputDriver drv;
    const LogicalSignalOutputs sideGreenOut = TrafficController::deriveOutputsForState(TrafficState::SIDE_GREEN);
    ElectricalPinStates elec;

    // Case A: Unverified Safe Disabled -> Logical is ON, but GPIO, ULN2003A sink, and Physical Lamp are OFF
    drv.computeElectricalStates(sideGreenOut, elec);
    TEST_ASSERT_TRUE(elec.isLogicalSignalOnForGpio(13));  // Layer 1: Side Green logical ON (GPIO 13)
    TEST_ASSERT_FALSE(elec.isGpioDrivenHigh(13));         // Layer 2: ESP32 GPIO 13 driven LOW
    TEST_ASSERT_FALSE(elec.isUlnSinkActiveForGpio(13));   // Layer 3: ULN2003A sink OFF
    TEST_ASSERT_FALSE(elec.isLampEnergizedForGpio(13));   // Layer 4: Physical lamp OFF

    // Case B: ULN2003A Low-Side Active-HIGH -> Logical ON => GPIO HIGH => ULN2003A Sink ON => Lamp ON
    drv.setVehicleDriverConfig(TrafficConfig::OutputDriverType::LOW_SIDE_ULN2003A_ACTIVE_HIGH, true);
    drv.computeElectricalStates(sideGreenOut, elec);
    TEST_ASSERT_TRUE(elec.isLogicalSignalOnForGpio(13));  // Layer 1: Side Green logical ON (GPIO 13)
    TEST_ASSERT_TRUE(elec.isGpioDrivenHigh(13));          // Layer 2: ESP32 GPIO 13 driven HIGH
    TEST_ASSERT_TRUE(elec.isUlnSinkActiveForGpio(13));    // Layer 3: ULN2003A collector sinks to GND
    TEST_ASSERT_TRUE(elec.isLampEnergizedForGpio(13));    // Layer 4: Physical lamp energized
    TEST_ASSERT_FALSE(elec.isLampEnergizedForGpio(14));   // Side Red (GPIO 14) OFF
    TEST_ASSERT_TRUE(elec.isLampEnergizedForGpio(27));    // Main Red (GPIO 27) ON
    TEST_ASSERT_TRUE(elec.isLampEnergizedForGpio(22));    // Turn Red (GPIO 22) ON
}

// 36. Test RHT Configuration Validation & RHT Movement Definitions
void test_rht_configuration_and_movement_definitions(void) {
    TEST_ASSERT_TRUE(TrafficConfig::RIGHT_HAND_TRAFFIC_RULE);
    TEST_ASSERT_EQUAL_STRING("RHT_KEEPRIGHT", TrafficConfig::TRAFFIC_RULE_ID);

    TrafficController ctrl;
    ctrl.begin(0);
    TEST_ASSERT_TRUE(ctrl.getPhaseManager().isRightHandTrafficRule());
    TEST_ASSERT_TRUE(ctrl.getConflictMatrixConfig().rightHandTrafficRule);

    // Verify all 5 RHT movement descriptors
    const auto& mainDesc = TrafficPhaseManager::getMovementDescriptor(TrafficConfig::MovementId::MAIN_ROAD);
    const auto& sideDesc = TrafficPhaseManager::getMovementDescriptor(TrafficConfig::MovementId::SIDE_ROAD);
    const auto& turnDesc = TrafficPhaseManager::getMovementDescriptor(TrafficConfig::MovementId::PROTECTED_TURN);
    const auto& ped1Desc = TrafficPhaseManager::getMovementDescriptor(TrafficConfig::MovementId::PED_CROSSING_1);
    const auto& ped2Desc = TrafficPhaseManager::getMovementDescriptor(TrafficConfig::MovementId::PED_CROSSING_2);

    TEST_ASSERT_TRUE(mainDesc.isVehicleMovement);
    TEST_ASSERT_FALSE(mainDesc.isCrossTrafficTurnUnderRht);
    TEST_ASSERT_TRUE(sideDesc.isVehicleMovement);
    TEST_ASSERT_FALSE(sideDesc.isCrossTrafficTurnUnderRht);
    TEST_ASSERT_TRUE(turnDesc.isVehicleMovement);
    TEST_ASSERT_TRUE(turnDesc.isCrossTrafficTurnUnderRht);
    TEST_ASSERT_FALSE(ped1Desc.isVehicleMovement);
    TEST_ASSERT_FALSE(ped2Desc.isVehicleMovement);

    // Verify PersistentControllerConfig rejects non-RHT configuration
    PersistentControllerConfig cfg = ctrl.exportPersistentConfig();
    TEST_ASSERT_TRUE(ConfigurationStorage::validateConfig(cfg));
    cfg.conflictMatrix.rightHandTrafficRule = false;
    cfg.checksum = ConfigurationStorage::computeChecksum(cfg);
    TEST_ASSERT_FALSE(ConfigurationStorage::validateConfig(cfg));
}

// 37. Test RHT Protected-Turn to Side-Road and Protected-Turn to Main-Road Transitions
void test_rht_protected_turn_to_side_and_turn_to_main_transitions(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.setControlMode(0, ControlMode::ACTUATED);
    ctrl.setSimulationMode(0, true);
    ctrl.setProtectedTurnCommissioned(0, true);

    ctrl.update(1000); // STARTUP_SAFE -> ALL_RED
    ctrl.update(2000); // ALL_RED -> MAIN_GREEN
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());

    // Inject Protected Turn demand and Side Road demand while Main Road has NO demand
    TEST_ASSERT_TRUE(ctrl.setSimMovementDemand(2500, TrafficConfig::MovementId::PROTECTED_TURN, true));
    ctrl.update(6000); // MAIN_GREEN -> MAIN_YELLOW
    ctrl.update(8000); // MAIN_YELLOW -> ALL_RED
    ctrl.update(9000); // ALL_RED -> TURN_GREEN
    TEST_ASSERT_EQUAL(TrafficState::TURN_GREEN, ctrl.getState());

    // While in TURN_GREEN, Side Road demand arrives and Turn demand clears
    TEST_ASSERT_TRUE(ctrl.setSimMovementDemand(9200, TrafficConfig::MovementId::SIDE_ROAD, true));
    TEST_ASSERT_TRUE(ctrl.setSimMovementDemand(9500, TrafficConfig::MovementId::PROTECTED_TURN, false));

    // At 11500 ms (turnMinGreenMs + turnClearGapMs elapsed), TURN_GREEN -> TURN_YELLOW
    ctrl.update(11500);
    TEST_ASSERT_EQUAL(TrafficState::TURN_YELLOW, ctrl.getState());
    TEST_ASSERT_EQUAL(NextPhaseTarget::SIDE_ROAD, ctrl.getNextTarget());

    // At 13500 ms -> ALL_RED, then at 14500 ms -> SIDE_GREEN (safe Turn-to-Side transition via Yellow + All-Red!)
    ctrl.update(13500);
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());
    ctrl.update(14500);
    TEST_ASSERT_EQUAL(TrafficState::SIDE_GREEN, ctrl.getState());

    // Clear Side Road demand -> transitions back to MAIN_GREEN via SIDE_YELLOW + ALL_RED
    TEST_ASSERT_TRUE(ctrl.setSimMovementDemand(15000, TrafficConfig::MovementId::SIDE_ROAD, false));
    ctrl.update(17000);
    TEST_ASSERT_EQUAL(TrafficState::SIDE_YELLOW, ctrl.getState());
    ctrl.update(19000);
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());
    ctrl.update(20000);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());
}

// 38. Test RHT Conflict Matrix & Pedestrian Crossing Relationships
void test_rht_conflict_matrix_and_pedestrian_crossing_relationships(void) {
    TrafficPhaseManager pm;
    pm.setProtectedTurnCommissioned(true);

    // 1. Main vs Side, Main vs Turn, Side vs Turn must all conflict under RHT
    TEST_ASSERT_TRUE(pm.areMovementsConflicting(TrafficConfig::MovementId::MAIN_ROAD, TrafficConfig::MovementId::SIDE_ROAD));
    TEST_ASSERT_TRUE(pm.areMovementsConflicting(TrafficConfig::MovementId::MAIN_ROAD, TrafficConfig::MovementId::PROTECTED_TURN));
    TEST_ASSERT_TRUE(pm.areMovementsConflicting(TrafficConfig::MovementId::SIDE_ROAD, TrafficConfig::MovementId::PROTECTED_TURN));

    // 2. Both pedestrian crossings must conflict with all 3 vehicle movements
    TEST_ASSERT_TRUE(pm.areMovementsConflicting(TrafficConfig::MovementId::PED_CROSSING_1, TrafficConfig::MovementId::MAIN_ROAD));
    TEST_ASSERT_TRUE(pm.areMovementsConflicting(TrafficConfig::MovementId::PED_CROSSING_1, TrafficConfig::MovementId::SIDE_ROAD));
    TEST_ASSERT_TRUE(pm.areMovementsConflicting(TrafficConfig::MovementId::PED_CROSSING_1, TrafficConfig::MovementId::PROTECTED_TURN));
    TEST_ASSERT_TRUE(pm.areMovementsConflicting(TrafficConfig::MovementId::PED_CROSSING_2, TrafficConfig::MovementId::MAIN_ROAD));
    TEST_ASSERT_TRUE(pm.areMovementsConflicting(TrafficConfig::MovementId::PED_CROSSING_2, TrafficConfig::MovementId::SIDE_ROAD));
    TEST_ASSERT_TRUE(pm.areMovementsConflicting(TrafficConfig::MovementId::PED_CROSSING_2, TrafficConfig::MovementId::PROTECTED_TURN));

    // 3. Pedestrian Crossing 1 and Pedestrian Crossing 2 are compatible when all vehicle movements are RED
    TEST_ASSERT_FALSE(pm.areMovementsConflicting(TrafficConfig::MovementId::PED_CROSSING_1, TrafficConfig::MovementId::PED_CROSSING_2));
    const bool bothPedsOnly[5] = { false, false, false, true, true };
    TEST_ASSERT_TRUE(pm.verifyActiveMovementsConflictFree(bothPedsOnly));

    // 4. Any vehicle movement concurrent with any pedestrian crossing is rejected
    const bool turnAndPed1[5] = { false, false, true, true, false };
    TEST_ASSERT_FALSE(pm.verifyActiveMovementsConflictFree(turnAndPed1));
}

// 39. Test Dashboard Status Snapshot & 13-Channel Telemetry Parsing
void test_dashboard_status_snapshot_and_13_channel_telemetry_parsing(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.setSimulationMode(0, true);
    ctrl.setProtectedTurnCommissioned(0, true);
    ctrl.update(1000); // -> ALL_RED
    ctrl.update(2000); // -> MAIN_GREEN

    const DashboardStatusSnapshot snapMain = SerialCommandHandler::captureStatusSnapshot(2500, ctrl);
    TEST_ASSERT_TRUE(snapMain.valid);
    TEST_ASSERT_TRUE(snapMain.rightHandTrafficRule);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, snapMain.currentFsmState);
    TEST_ASSERT_EQUAL_UINT32(500, snapMain.stateElapsedMs);
    TEST_ASSERT_TRUE(snapMain.outputs.mainGreen);
    TEST_ASSERT_TRUE(snapMain.outputs.sideRed);
    TEST_ASSERT_TRUE(snapMain.outputs.turnRed);
    TEST_ASSERT_TRUE(snapMain.outputs.pedDontWalk);
    TEST_ASSERT_TRUE(snapMain.outputs.ped2DontWalk);

    char lineBuf[128];
    SerialCommandHandler::formatLogicalOutputsLine(snapMain.outputs, lineBuf, sizeof(lineBuf));
    TEST_ASSERT_EQUAL_STRING(
        "MAIN[G=1 Y=0 R=0] SIDE[G=0 Y=0 R=1] TURN[G=0 Y=0 R=1] PED1[W=0 DW=1] PED2[W=0 DW=1]",
        lineBuf);

    SignalOutputs parsed{};
    TEST_ASSERT_TRUE(SerialCommandHandler::parseLogicalOutputsLine(lineBuf, parsed));
    TEST_ASSERT_TRUE(parsed.mainGreen);
    TEST_ASSERT_FALSE(parsed.mainYellow);
    TEST_ASSERT_FALSE(parsed.mainRed);
    TEST_ASSERT_TRUE(parsed.sideRed);
    TEST_ASSERT_TRUE(parsed.turnRed);
    TEST_ASSERT_TRUE(parsed.pedDontWalk);
    TEST_ASSERT_TRUE(parsed.ped2DontWalk);
}

// 40. Test Runtime Driver Commissioning & 5V Common-Ground Vehicle Interlock
void test_runtime_driver_commissioning_and_common_ground_interlock(void) {
    TrafficController ctrl;
    ctrl.getStorage().setUseHardwareNvs(false);
    ctrl.begin(0);
    SerialCommandHandler cmd;

    // 1. Default topologies match confirmed wiring (DIRECT_ACTIVE_HIGH for vehicle, LOW_SIDE_ULN2003A_ACTIVE_HIGH for ped), both unverified
    TEST_ASSERT_EQUAL(TrafficConfig::OutputDriverType::DIRECT_ACTIVE_HIGH, ctrl.getOutputDriver().getVehicleDriverType());
    TEST_ASSERT_EQUAL(TrafficConfig::OutputDriverType::LOW_SIDE_ULN2003A_ACTIVE_HIGH, ctrl.getOutputDriver().getPedestrianDriverType());
    TEST_ASSERT_FALSE(ctrl.getOutputDriver().isVehicleDriverVerified());
    TEST_ASSERT_FALSE(ctrl.getOutputDriver().isPedestrianDriverVerified());

    // 2. Commissioning vehicle driver while set to UNVERIFIED_SAFE_DISABLED or bare LOW_SIDE_ULN2003A must be REJECTED
    TEST_ASSERT_EQUAL(SerialCommandResult::OK_EXECUTED, cmd.executeCommand(80, "DRIVER VEH DISABLED", ctrl));
    TEST_ASSERT_EQUAL(SerialCommandResult::REJECTED_BY_STATE, cmd.executeCommand(100, "COMMISSION VEH ON", ctrl));
    TEST_ASSERT_FALSE(ctrl.getOutputDriver().isVehicleDriverVerified());

    TEST_ASSERT_EQUAL(SerialCommandResult::OK_EXECUTED, cmd.executeCommand(150, "DRIVER VEH LOW_SIDE_ULN2003A", ctrl));
    TEST_ASSERT_EQUAL(SerialCommandResult::REJECTED_BY_STATE, cmd.executeCommand(200, "COMMISSION VEH ON", ctrl));
    TEST_ASSERT_FALSE(ctrl.getOutputDriver().isVehicleDriverVerified());

    // 3. Configuring DIRECT_ACTIVE_HIGH (confirmed topology) or High-Side Sourcing allows commissioning vehicle driver
    TEST_ASSERT_EQUAL(SerialCommandResult::OK_EXECUTED, cmd.executeCommand(220, "DRIVER VEH DIRECT_ACTIVE_HIGH", ctrl));
    TEST_ASSERT_EQUAL(SerialCommandResult::OK_EXECUTED, cmd.executeCommand(230, "COMMISSION VEH ON", ctrl));
    TEST_ASSERT_TRUE(ctrl.getOutputDriver().isVehicleDriverVerified());

    TEST_ASSERT_EQUAL(SerialCommandResult::OK_EXECUTED, cmd.executeCommand(250, "DRIVER VEH HIGH_SIDE_ACTIVE_HIGH", ctrl));
    TEST_ASSERT_EQUAL(SerialCommandResult::OK_EXECUTED, cmd.executeCommand(300, "COMMISSION VEH ON", ctrl));
    TEST_ASSERT_TRUE(ctrl.getOutputDriver().isVehicleDriverVerified());

    // 4. Pedestrian driver defaults to LOW_SIDE_ULN2003A_ACTIVE_HIGH and can be commissioned
    TEST_ASSERT_EQUAL(SerialCommandResult::OK_EXECUTED, cmd.executeCommand(350, "COMMISSION PED ON", ctrl));
    TEST_ASSERT_TRUE(ctrl.getOutputDriver().isPedestrianDriverVerified());
}

// 41. Test Single-Lamp Test Mode Safety & Unverified Driver Interlock
void test_single_lamp_test_mode_safety_and_interlocks(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    SerialCommandHandler cmd;

    // 1. In STARTUP_SAFE/MANUAL, LAMP TEST must still reject unverified drivers!
    ctrl.setControlMode(0, ControlMode::MANUAL);
    TEST_ASSERT_EQUAL(SerialCommandResult::REJECTED_BY_STATE, cmd.executeCommand(100, "LAMP TEST MAIN_R", ctrl));
    TEST_ASSERT_FALSE(ctrl.getOutputDriver().isLampTestActive());

    // 2. Commission high-side vehicle driver and test MAIN_R (channel 2, GPIO 27)
    ctrl.configureVehicleDriver(150, TrafficConfig::OutputDriverType::HIGH_SIDE_SOURCING_ACTIVE_HIGH);
    ctrl.commissionVehicleDriver(160, true);
    TEST_ASSERT_EQUAL(SerialCommandResult::OK_EXECUTED, cmd.executeCommand(200, "LAMP TEST MAIN_R", ctrl));
    TEST_ASSERT_TRUE(ctrl.getOutputDriver().isLampTestActive());
    TEST_ASSERT_EQUAL_INT8(2, ctrl.getOutputDriver().getLampTestChannel());

    ElectricalPinStates elec;
    ctrl.getOutputDriver().computeElectricalStates(SignalOutputDriver::buildSingleLampLogicalOutputs(2), elec);
    TEST_ASSERT_TRUE(elec.isLampEnergizedForGpio(27));
    TEST_ASSERT_FALSE(elec.isLampEnergizedForGpio(25));
    TEST_ASSERT_FALSE(elec.isLampEnergizedForGpio(14));
    TEST_ASSERT_FALSE(elec.isLampEnergizedForGpio(22));

    // 3. Triggering FAULT immediately cancels lamp test and forces all outputs OFF
    ctrl.triggerFault(300, "TEST_LAMP_FAULT", FaultCode::OPERATOR_INJECTED_FAULT);
    TEST_ASSERT_FALSE(ctrl.getOutputDriver().isLampTestActive());
}

// 42. Test IR Sensor Vehicle Release Detection & Per-Channel Electrical Health Commissioning
void test_ir_sensor_release_detection_and_per_channel_commissioning(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    SerialCommandHandler cmd;

    // 1. Before sampling physical pins, COMMISSION IR 1 ON is rejected (NO_PIN_SAMPLE_YET)
    TEST_ASSERT_EQUAL(SerialCommandResult::REJECTED_BY_STATE, cmd.executeCommand(50, "COMMISSION IR 1 ON", ctrl));

    // 2. Sample healthy idle-HIGH pins across all 6 IR sensors:
    //    COMMISSION IR 1 ON must STILL be rejected because idle HIGH alone is insufficient without a detect->release cycle!
    bool irHigh[6] = { true, true, true, true, true, true };
    ctrl.processAllPhysicalInputs(100, irHigh, true, true);
    TEST_ASSERT_EQUAL(SerialCommandResult::REJECTED_BY_STATE, cmd.executeCommand(120, "COMMISSION IR 1 ON", ctrl));

    // 3. Perform physical detect (LOW) and release (HIGH) pre-commissioning cycle on IR 1 (GPIO 33) and IR 3 (GPIO 35)
    irHigh[0] = false;
    irHigh[2] = false;
    ctrl.processAllPhysicalInputs(130, irHigh, true, true);
    irHigh[0] = true;
    irHigh[2] = true;
    ctrl.processAllPhysicalInputs(140, irHigh, true, true);
    TEST_ASSERT_TRUE(ctrl.getVehicleDetector().getChannel(0).cycleObserved);
    TEST_ASSERT_TRUE(ctrl.getVehicleDetector().getChannel(2).cycleObserved);

    // 4. Now COMMISSION IR 1 (GPIO 33) and IR 3 (GPIO 35) individually succeed, while IR 2 (no cycle) is rejected
    TEST_ASSERT_EQUAL(SerialCommandResult::OK_EXECUTED, cmd.executeCommand(150, "COMMISSION IR 1 ON", ctrl));
    TEST_ASSERT_EQUAL(SerialCommandResult::OK_EXECUTED, cmd.executeCommand(160, "COMMISSION IR 3 ON", ctrl));
    TEST_ASSERT_EQUAL(SerialCommandResult::REJECTED_BY_STATE, cmd.executeCommand(170, "COMMISSION IR 2 ON", ctrl));
    TEST_ASSERT_TRUE(ctrl.isPhysicalIrValidated());
    TEST_ASSERT_FALSE(ctrl.isAllPhysicalIrValidated());
    TEST_ASSERT_EQUAL_UINT8(0x05, ctrl.getVehicleDetector().getValidatedMask());

    // 5. Simulate vehicle arrival (active-LOW for >= 120 ms) and subsequent vehicle release on IR 3 (GPIO 35)
    irHigh[2] = false;
    ctrl.processAllPhysicalInputs(200, irHigh, true, true);
    ctrl.processAllPhysicalInputs(350, irHigh, true, true);
    TEST_ASSERT_TRUE(ctrl.getVehicleDetector().getChannel(2).stableDetected);
    TEST_ASSERT_EQUAL_UINT32(1, ctrl.getVehicleDetector().getChannel(2).detectionCount);
    TEST_ASSERT_EQUAL_UINT32(0, ctrl.getVehicleDetector().getChannel(2).releaseCount);

    // Vehicle departs (returns HIGH for >= 120 ms)
    irHigh[2] = true;
    ctrl.processAllPhysicalInputs(400, irHigh, true, true);
    ctrl.processAllPhysicalInputs(550, irHigh, true, true);
    TEST_ASSERT_FALSE(ctrl.getVehicleDetector().getChannel(2).stableDetected);
    TEST_ASSERT_EQUAL_UINT32(1, ctrl.getVehicleDetector().getChannel(2).detectionCount);
    TEST_ASSERT_EQUAL_UINT32(1, ctrl.getVehicleDetector().getChannel(2).releaseCount);
}

// 43. Test Adaptive Pedestrian Waiting-Time Priority & Starvation Prevention
void test_adaptive_pedestrian_waiting_priority_and_starvation_boost(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    // Unvalidated IR sensors reject ADAPTIVE mode
    TEST_ASSERT_FALSE(ctrl.setControlMode(0, ControlMode::ADAPTIVE));
    ctrl.setPhysicalIrValidated(true);
    TEST_ASSERT_TRUE(ctrl.setControlMode(0, ControlMode::ADAPTIVE));
    ctrl.setProtectedTurnCommissioned(0, true);
    ctrl.update(1000); // -> ALL_RED
    ctrl.update(2000); // -> MAIN_GREEN

    // Side Road (2 sensors: IR3+IR4) and Protected Turn (2 sensors: IR5+IR6) both have heavy queues (score=6 each),
    // while Ped 1 button is pressed (score=3+1=4).
    bool irHigh[6] = { true, true, false, false, false, false };
    ctrl.processAllPhysicalInputs(2100, irHigh, false, true); // PED 1 pressed (LOW)
    ctrl.processAllPhysicalInputs(2160, irHigh, false, true); // PED 1 held >= 50 ms debounce -> latched!
    ctrl.processAllPhysicalInputs(2250, irHigh, true, true);  // PED 1 released, Side S1+S2 and Turn S1+S2 stable
    TEST_ASSERT_TRUE(ctrl.hasPedestrianChannelDemand(0));

    // Cycle 1: At 6000 ms, Turn (2 sensors = 6) beats Ped 1 (4) -> serves TURN_GREEN first
    ctrl.update(6000); // -> MAIN_YELLOW
    TEST_ASSERT_EQUAL(NextPhaseTarget::PROTECTED_TURN, ctrl.getNextTarget());
    ctrl.update(8000); // -> ALL_RED
    ctrl.update(9000); // -> TURN_GREEN (Ped 1 skippedCycles becomes 1, Side skippedCycles becomes 1)
    TEST_ASSERT_EQUAL(TrafficState::TURN_GREEN, ctrl.getState());
    TEST_ASSERT_EQUAL_UINT8(1, ctrl.getPedestrianController().getMaxSkippedCycles());

    // Keep sensors active until Turn Max Green (6000 ms -> t=15000 ms)
    ctrl.processAllPhysicalInputs(14500, irHigh, true, true);
    ctrl.update(15000); // -> TURN_YELLOW (Side Road has 2 sensors + wait, beating Ped 1 which has 1 skip < limit 2)
    TEST_ASSERT_EQUAL(NextPhaseTarget::SIDE_ROAD, ctrl.getNextTarget());
    ctrl.update(17000); // -> ALL_RED
    ctrl.update(18000); // -> SIDE_GREEN (Ped 1 skippedCycles becomes 2 >= starvationSkipLimit!)
    TEST_ASSERT_EQUAL(TrafficState::SIDE_GREEN, ctrl.getState());
    TEST_ASSERT_TRUE(ctrl.getPedestrianController().getMaxSkippedCycles() >= 2);

    // At Side Max Green (8000 ms -> t=26000 ms), Ped 1 has starvation boost (+25 pts) and MUST beat Turn & Main!
    ctrl.processAllPhysicalInputs(25500, irHigh, true, true);
    ctrl.update(26000); // -> SIDE_YELLOW
    TEST_ASSERT_EQUAL(NextPhaseTarget::PEDESTRIAN, ctrl.getNextTarget());
    ctrl.update(28000); // -> ALL_RED
    ctrl.update(29000); // -> PED_WALK
    TEST_ASSERT_EQUAL(TrafficState::PED_WALK, ctrl.getState());
}

// 44. Test Manual Mode Rejects NEXT During Clearance Transitions
void test_manual_mode_clearance_step_rejection(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.update(1000); // -> ALL_RED
    ctrl.update(2000); // -> MAIN_GREEN
    ctrl.setControlMode(2000, ControlMode::MANUAL);

    // 1. Trigger NEXT after mainMinGreenMs (6000 ms) -> enters MAIN_YELLOW
    TEST_ASSERT_TRUE(ctrl.advanceManualStep(6000));
    ctrl.update(6000);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_YELLOW, ctrl.getState());

    // 2. Sending NEXT while in MAIN_YELLOW or ALL_RED must be REJECTED so it cannot queue a premature jump
    TEST_ASSERT_FALSE(ctrl.advanceManualStep(6500));
    ctrl.update(8000); // -> ALL_RED
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());
    TEST_ASSERT_FALSE(ctrl.advanceManualStep(8500));

    // 3. Enters SIDE_GREEN at 9000 ms and remains there until a fresh NEXT is sent
    ctrl.update(9000);
    TEST_ASSERT_EQUAL(TrafficState::SIDE_GREEN, ctrl.getState());
    ctrl.update(20000);
    TEST_ASSERT_EQUAL(TrafficState::SIDE_GREEN, ctrl.getState());
}

// 45. Test First-Boot NVS Initialization (No False Fault) & Commissioning Persistence
void test_nvs_first_boot_initialization_and_commissioning_persistence(void) {
    TrafficController ctrl;
    ctrl.getStorage().setUseHardwareNvs(false);
    ctrl.begin(0);

    // 1. First boot with uninitialized NVS initializes defaults cleanly WITHOUT setting 0x0010 INVALID_CONFIGURATION fault
    TEST_ASSERT_TRUE(ctrl.loadConfigFromNvs(50));
    TEST_ASSERT_EQUAL_HEX16(0x0000, ctrl.getFaultManager().getActiveFaultMask());
    TEST_ASSERT_EQUAL_STRING("NONE", ctrl.getFaultReason());

    // 2. Commission high-side vehicle driver, pedestrian driver, and RHT protected turn, then SAVE to NVS
    ctrl.configureVehicleDriver(100, TrafficConfig::OutputDriverType::HIGH_SIDE_SOURCING_ACTIVE_HIGH);
    TEST_ASSERT_TRUE(ctrl.commissionVehicleDriver(110, true));
    TEST_ASSERT_TRUE(ctrl.commissionPedestrianDriver(120, true));
    TEST_ASSERT_TRUE(ctrl.commissionProtectedTurn(130, true));
    TEST_ASSERT_TRUE(ctrl.saveConfigToNvs(150));

    // 3. Decommission settings in RAM, then LOAD from NVS and verify all commissioned settings persist
    ctrl.commissionVehicleDriver(200, false);
    ctrl.commissionPedestrianDriver(200, false);
    ctrl.commissionProtectedTurn(200, false);
    TEST_ASSERT_FALSE(ctrl.getOutputDriver().isVehicleDriverVerified());
    TEST_ASSERT_FALSE(ctrl.isProtectedTurnCommissioned());

    TEST_ASSERT_TRUE(ctrl.loadConfigFromNvs(250));
    TEST_ASSERT_TRUE(ctrl.getOutputDriver().isVehicleDriverVerified());
    TEST_ASSERT_TRUE(ctrl.getOutputDriver().isPedestrianDriverVerified());
    TEST_ASSERT_TRUE(ctrl.isProtectedTurnCommissioned());
}

// 46. Test Confirmed Physical Wiring Polarity: Vehicle DIRECT_ACTIVE_HIGH & Pedestrian LOW_SIDE_ULN2003A_ACTIVE_HIGH
void test_confirmed_direct_vehicle_and_uln2003a_pedestrian_polarity(void) {
    // Verify compile-time safety gates remain disabled by default
    TEST_ASSERT_FALSE(TrafficConfig::VEHICLE_HIGH_SIDE_DRIVER_VERIFIED);
    TEST_ASSERT_FALSE(TrafficConfig::PED_ULN2003A_DRIVER_VERIFIED);
    TEST_ASSERT_FALSE(TrafficConfig::PHYSICAL_IR_VALIDATED);
    TEST_ASSERT_FALSE(TrafficConfig::PROTECTED_TURN_GEOMETRY_VERIFIED);

    SignalOutputDriver drv;
    TEST_ASSERT_EQUAL(TrafficConfig::OutputDriverType::DIRECT_ACTIVE_HIGH, drv.getVehicleDriverType());
    TEST_ASSERT_EQUAL(TrafficConfig::OutputDriverType::LOW_SIDE_ULN2003A_ACTIVE_HIGH, drv.getPedestrianDriverType());
    TEST_ASSERT_EQUAL_STRING("DIRECT_ACTIVE_HIGH", SignalOutputDriver::driverTypeToString(drv.getVehicleDriverType()));
    TEST_ASSERT_EQUAL_STRING("LOW_SIDE_ULN2003A_ACTIVE_HIGH", SignalOutputDriver::driverTypeToString(drv.getPedestrianDriverType()));
    TEST_ASSERT_FALSE(drv.isVehicleDriverVerified());
    TEST_ASSERT_FALSE(drv.isPedestrianDriverVerified());

    // 1. While unverified (default), all 9 vehicle GPIOs and 4 pedestrian GPIOs stay LOW (0V) and unenergized
    const LogicalSignalOutputs mainGreen = TrafficController::deriveOutputsForState(TrafficState::MAIN_GREEN);
    ElectricalPinStates elec;
    drv.computeElectricalStates(mainGreen, elec);
    for (uint8_t i = 0; i < TrafficConfig::TOTAL_SIGNAL_OUTPUTS; ++i) {
        TEST_ASSERT_FALSE(elec.pinHigh[i]);
        TEST_ASSERT_FALSE(elec.ulnSinkActive[i]);
        TEST_ASSERT_FALSE(elec.channelEnergized[i]);
    }

    // 2. When Vehicle DIRECT_ACTIVE_HIGH is commissioned:
    //    Logical ON  -> ESP32 GPIO HIGH (true), ulnSinkActive = false (direct GPIO), channelEnergized = true
    //    Logical OFF -> ESP32 GPIO LOW (false), ulnSinkActive = false, channelEnergized = false
    drv.setVehicleDriverConfig(TrafficConfig::OutputDriverType::DIRECT_ACTIVE_HIGH, true);
    for (uint8_t ch = 0; ch < 9; ++ch) {
        const LogicalSignalOutputs singleVeh = SignalOutputDriver::buildSingleLampLogicalOutputs(static_cast<int8_t>(ch));
        drv.computeElectricalStates(singleVeh, elec);
        for (uint8_t i = 0; i < 9; ++i) {
            const bool expectOn = (i == ch);
            TEST_ASSERT_EQUAL(expectOn, elec.logicalState[i]);
            TEST_ASSERT_EQUAL(expectOn, elec.pinHigh[i]);
            TEST_ASSERT_FALSE(elec.ulnSinkActive[i]); // Vehicle channels are direct GPIO, NOT ULN2003A
            TEST_ASSERT_EQUAL(expectOn, elec.channelEnergized[i]);
        }
        // Pedestrian channels remain unverified -> all LOW/OFF
        for (uint8_t p = 9; p < 13; ++p) {
            TEST_ASSERT_FALSE(elec.pinHigh[p]);
            TEST_ASSERT_FALSE(elec.ulnSinkActive[p]);
            TEST_ASSERT_FALSE(elec.channelEnergized[p]);
        }
    }

    // 3. When Pedestrian LOW_SIDE_ULN2003A_ACTIVE_HIGH is commissioned:
    //    Logical ON  -> ESP32 GPIO HIGH (true), ulnSinkActive = true (ULN2003A sinks LOW to GND), channelEnergized = true
    //    Logical OFF -> ESP32 GPIO LOW (false), ulnSinkActive = false, channelEnergized = false
    drv.setPedestrianDriverConfig(TrafficConfig::OutputDriverType::LOW_SIDE_ULN2003A_ACTIVE_HIGH, true);
    for (uint8_t ch = 9; ch < 13; ++ch) {
        const LogicalSignalOutputs singlePed = SignalOutputDriver::buildSingleLampLogicalOutputs(static_cast<int8_t>(ch));
        drv.computeElectricalStates(singlePed, elec);
        for (uint8_t p = 9; p < 13; ++p) {
            const bool expectOn = (p == ch);
            TEST_ASSERT_EQUAL(expectOn, elec.logicalState[p]);
            TEST_ASSERT_EQUAL(expectOn, elec.pinHigh[p]);
            TEST_ASSERT_EQUAL(expectOn, elec.ulnSinkActive[p]); // ULN2003A low-side sink active when GPIO HIGH
            TEST_ASSERT_EQUAL(expectOn, elec.channelEnergized[p]);
        }
    }

    // 4. Verify persistent default configuration also stores DIRECT_ACTIVE_HIGH (veh) and LOW_SIDE_ULN2003A_ACTIVE_HIGH (ped)
    const PersistentControllerConfig defCfg = ConfigurationStorage::makeDefaultConfig();
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(TrafficConfig::OutputDriverType::DIRECT_ACTIVE_HIGH), defCfg.vehicleDriverType);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(TrafficConfig::OutputDriverType::LOW_SIDE_ULN2003A_ACTIVE_HIGH), defCfg.pedDriverType);
    TEST_ASSERT_FALSE(defCfg.vehicleDriverVerified);
    TEST_ASSERT_FALSE(defCfg.pedDriverVerified);
}

// 47. Test Comprehensive Fault Architecture, Serial CLI Command Rejection in FAULT_SAFE, and Recovery Sequencing
void test_comprehensive_fault_architecture_and_recovery_sequencing(void) {
    TrafficController ctrl;
    ctrl.getStorage().setUseHardwareNvs(false);
    ctrl.begin(0);
    SerialCommandHandler cmd;

    ctrl.update(1000); // -> ALL_RED
    ctrl.update(2000); // -> MAIN_GREEN
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());

    // 1. Inject Critical Fault
    ctrl.triggerFault(2500, "TEST_CRITICAL_SAFETY_FAULT", FaultCode::OPERATOR_INJECTED_FAULT);
    TEST_ASSERT_EQUAL(TrafficState::FAULT_SAFE, ctrl.getState());
    TEST_ASSERT_TRUE(ctrl.getFaultManager().isCriticalFaultLatched());

    // 2. All 13 outputs must be completely OFF
    const SignalOutputs faultOut = ctrl.getOutputs();
    TEST_ASSERT_FALSE(faultOut.mainGreen || faultOut.mainYellow || faultOut.mainRed);
    TEST_ASSERT_FALSE(faultOut.sideGreen || faultOut.sideYellow || faultOut.sideRed);
    TEST_ASSERT_FALSE(faultOut.turnGreen || faultOut.turnYellow || faultOut.turnRed);
    TEST_ASSERT_FALSE(faultOut.pedWalk || faultOut.pedDontWalk);
    TEST_ASSERT_FALSE(faultOut.ped2Walk || faultOut.ped2DontWalk);

    // 3. Operational commands are strictly REJECTED while in FAULT_SAFE
    TEST_ASSERT_EQUAL(SerialCommandResult::REJECTED_BY_STATE, cmd.executeCommand(2600, "NEXT", ctrl));
    TEST_ASSERT_EQUAL(SerialCommandResult::REJECTED_BY_STATE, cmd.executeCommand(2610, "MODE AUTO", ctrl));
    TEST_ASSERT_EQUAL(SerialCommandResult::REJECTED_BY_STATE, cmd.executeCommand(2620, "MODE MANUAL", ctrl));
    TEST_ASSERT_EQUAL(SerialCommandResult::REJECTED_BY_STATE, cmd.executeCommand(2630, "CAR ON", ctrl));
    TEST_ASSERT_EQUAL(SerialCommandResult::REJECTED_BY_STATE, cmd.executeCommand(2640, "CAR MAIN ON", ctrl));
    TEST_ASSERT_EQUAL(SerialCommandResult::REJECTED_BY_STATE, cmd.executeCommand(2650, "PED 1", ctrl));
    TEST_ASSERT_EQUAL(SerialCommandResult::REJECTED_BY_STATE, cmd.executeCommand(2660, "LAMP TEST MAIN_R", ctrl));
    TEST_ASSERT_EQUAL(SerialCommandResult::REJECTED_BY_STATE, cmd.executeCommand(2670, "COMMISSION VEH ON", ctrl));
    TEST_ASSERT_EQUAL(SerialCommandResult::REJECTED_BY_STATE, cmd.executeCommand(2680, "COMMISSION TURN ON", ctrl));

    // 4. Recovering via CLEAR: Must NEVER jump directly to Green!
    TEST_ASSERT_EQUAL(SerialCommandResult::OK_EXECUTED, cmd.executeCommand(2700, "CLEAR", ctrl));
    TEST_ASSERT_EQUAL(TrafficState::STARTUP_SAFE, ctrl.getState());
    TEST_ASSERT_FALSE(ctrl.getFaultManager().isCriticalFaultLatched());

    // After 999 ms, still in STARTUP_SAFE
    ctrl.update(3699);
    TEST_ASSERT_EQUAL(TrafficState::STARTUP_SAFE, ctrl.getState());

    // At 1000 ms, transitions to ALL_RED clearance (never straight to green)
    ctrl.update(3700);
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());
    const SignalOutputs allRedOut = ctrl.getOutputs();
    TEST_ASSERT_TRUE(allRedOut.mainRed && allRedOut.sideRed && allRedOut.turnRed);
    TEST_ASSERT_TRUE(allRedOut.pedDontWalk && allRedOut.ped2DontWalk);
    TEST_ASSERT_FALSE(allRedOut.mainGreen || allRedOut.sideGreen || allRedOut.turnGreen);

    // After 1000 ms all-red clearance, safely enters MAIN_GREEN
    ctrl.update(4700);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());

    // 5. Verify Pedestrian Request Isolation (Requesting PED2 only keeps PED1 at DONT_WALK)
    ctrl.setSimulationMode(4700, true);
    ctrl.triggerSimPedestrianChannel(4800, 1); // Request only PED2 (Channel 1)
    TEST_ASSERT_FALSE(ctrl.hasPedestrianChannelDemand(0));
    TEST_ASSERT_TRUE(ctrl.hasPedestrianChannelDemand(1));

    // Progress through Main Yellow -> All-Red -> PED_WALK
    ctrl.update(8700);  // -> MAIN_YELLOW
    ctrl.update(10700); // -> ALL_RED
    ctrl.update(11700); // -> PED_WALK
    TEST_ASSERT_EQUAL(TrafficState::PED_WALK, ctrl.getState());

    const SignalOutputs pedWalkOut = ctrl.getOutputs();
    // Only PED2 was requested: PED2 must show WALK, while PED1 strictly remains DONT_WALK!
    TEST_ASSERT_FALSE(pedWalkOut.pedWalk);
    TEST_ASSERT_TRUE(pedWalkOut.pedDontWalk);
    TEST_ASSERT_TRUE(pedWalkOut.ped2Walk);
    TEST_ASSERT_FALSE(pedWalkOut.ped2DontWalk);
    // All vehicle movements must remain RED
    TEST_ASSERT_TRUE(pedWalkOut.mainRed && pedWalkOut.sideRed && pedWalkOut.turnRed);
    TEST_ASSERT_FALSE(pedWalkOut.mainGreen || pedWalkOut.mainYellow);
    TEST_ASSERT_FALSE(pedWalkOut.sideGreen || pedWalkOut.sideYellow);
    TEST_ASSERT_FALSE(pedWalkOut.turnGreen || pedWalkOut.turnYellow);
}

// 48. Test Fixed-Time Prototype Timings, Continuous Sequence, and IR Sensor Isolation
void test_fixed_time_prototype_sequence_and_sensor_isolation(void) {
    TrafficController ctrl;
    ctrl.getStorage().setUseHardwareNvs(false);
    ctrl.begin(0);

    // Apply authoritative prototype fixed-time timings:
    // Main Green 25s, Main Yellow 3s, All-Red 2s, Side Green 20s, Side Yellow 3s, All-Red 2s
    ctrl.setTimingConfig(0, TrafficConfig::PROTOTYPE_TIMING_CONFIG);
    TEST_ASSERT_TRUE(ctrl.setControlMode(0, ControlMode::FIXED));
    TEST_ASSERT_EQUAL(ControlMode::FIXED, ctrl.getControlMode());

    // 1. Verify uncommissioned sensor isolation and ADAPTIVE rejection
    TEST_ASSERT_FALSE(ctrl.isAllPhysicalIrValidated());
    TEST_ASSERT_FALSE(ctrl.setControlMode(0, ControlMode::ADAPTIVE)); // Rejected!
    TEST_ASSERT_EQUAL(ControlMode::FIXED, ctrl.getControlMode());

    // 2. Initial state: STARTUP_SAFE (1000 ms)
    TEST_ASSERT_EQUAL(TrafficState::STARTUP_SAFE, ctrl.getState());
    ctrl.update(999);
    TEST_ASSERT_EQUAL(TrafficState::STARTUP_SAFE, ctrl.getState());
    ctrl.update(1000);
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());

    // ALL_RED startup clearance (PROTOTYPE_ALL_RED_MS = 2000 ms: from 1000 -> 3000 ms)
    ctrl.update(2999);
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());
    ctrl.update(3000);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());

    // 3. Step 1: MAIN_GREEN - 25 seconds (3000 -> 28000 ms)
    // Inject uncommissioned IR sensor inputs; verify they do NOT terminate or extend green
    bool irHigh[6] = { true, true, false, true, false, false }; // Side road sensor active
    ctrl.processAllPhysicalInputs(5000, irHigh, true, true);
    ctrl.update(15000);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());
    ctrl.update(27999);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());
    ctrl.update(28000); // 25s elapsed!
    TEST_ASSERT_EQUAL(TrafficState::MAIN_YELLOW, ctrl.getState());

    // 4. Step 2: MAIN_YELLOW - 3 seconds (28000 -> 31000 ms)
    ctrl.update(30999);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_YELLOW, ctrl.getState());
    ctrl.update(31000); // 3s elapsed!
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());

    // 5. Step 3: ALL_RED - 2 seconds (31000 -> 33000 ms)
    ctrl.update(32999);
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());
    ctrl.update(33000); // 2s elapsed!
    TEST_ASSERT_EQUAL(TrafficState::SIDE_GREEN, ctrl.getState());

    // 6. Step 4: SIDE_GREEN - 20 seconds (33000 -> 53000 ms)
    // Zero vehicle presence on side road; fixed-time MUST hold full 20s without early termination!
    bool irAllIdle[6] = { true, true, true, true, true, true };
    ctrl.processAllPhysicalInputs(35000, irAllIdle, true, true);
    ctrl.update(45000);
    TEST_ASSERT_EQUAL(TrafficState::SIDE_GREEN, ctrl.getState());
    ctrl.update(52999);
    TEST_ASSERT_EQUAL(TrafficState::SIDE_GREEN, ctrl.getState());
    ctrl.update(53000); // 20s elapsed!
    TEST_ASSERT_EQUAL(TrafficState::SIDE_YELLOW, ctrl.getState());

    // 7. Step 5: SIDE_YELLOW - 3 seconds (53000 -> 56000 ms)
    ctrl.update(55999);
    TEST_ASSERT_EQUAL(TrafficState::SIDE_YELLOW, ctrl.getState());
    ctrl.update(56000); // 3s elapsed!
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());

    // 8. Step 6: ALL_RED - 2 seconds (56000 -> 58000 ms)
    ctrl.update(57999);
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());
    ctrl.update(58000); // 2s elapsed!
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState()); // Continuous cycle repeats!

    // 9. Verify Protected Turn is completely disabled in fixed-time sequence
    TEST_ASSERT_FALSE(ctrl.isProtectedTurnCommissioned());
    SignalOutputs mainOut = ctrl.getOutputs();
    TEST_ASSERT_TRUE(mainOut.turnRed);
    TEST_ASSERT_FALSE(mainOut.turnGreen);
    TEST_ASSERT_FALSE(mainOut.turnYellow);
}

// 49. Test Adaptive Mode Sensor Fault Fallback to Fixed-Time Recall
void test_adaptive_mode_sensor_fault_fallback_recall(void) {
    TrafficController ctrl;
    ctrl.getStorage().setUseHardwareNvs(false);
    ctrl.begin(0);

    // Validate all IR sensors and enter ADAPTIVE mode
    ctrl.setPhysicalIrValidated(true);
    TEST_ASSERT_TRUE(ctrl.setControlMode(0, ControlMode::ADAPTIVE));
    TEST_ASSERT_EQUAL(ControlMode::ADAPTIVE, ctrl.getControlMode());

    ctrl.update(1000); // -> ALL_RED
    ctrl.update(2000); // -> MAIN_GREEN
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());

    // Active vehicle presence on Main Road extends MAIN_GREEN
    bool irMain[6] = { false, true, true, true, true, true };
    ctrl.processAllPhysicalInputs(2500, irMain, true, true);
    ctrl.processAllPhysicalInputs(2650, irMain, true, true);
    ctrl.update(5000);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());

    // Now inject a sensor fault (chatter fault on Side Road Sensor 1, IR3 / GPIO 35) while Main clears
    for (uint8_t i = 0; i < 15; ++i) {
        bool irPins[6] = { true, true, (i % 2 == 0), true, true, true };
        ctrl.processAllPhysicalInputs(5100 + i * 20, irPins, true, true);
    }
    TEST_ASSERT_TRUE(ctrl.getVehicleDetector().isAnyChannelFaulted());

    // With sensor faulted, adaptive controller detects fallback condition and recalls SIDE_ROAD!
    ctrl.update(6000); // -> MAIN_YELLOW
    TEST_ASSERT_EQUAL(TrafficState::MAIN_YELLOW, ctrl.getState());
    ctrl.update(8000); // -> ALL_RED
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());
    ctrl.update(9000); // -> SIDE_GREEN (fallback service granted!)
    TEST_ASSERT_EQUAL(TrafficState::SIDE_GREEN, ctrl.getState());
}

void test_all_green_diagnostic_10s_execution_and_all_red_hold(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.commissionVehicleDriver(50, true);
    ctrl.update(1000); // -> ALL_RED
    ctrl.update(3000); // -> MAIN_GREEN
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());

    // 1. Without explicit bench unlock, ALL-GREEN test MUST be rejected for junction safety
    const char* rejReason = nullptr;
    TEST_ASSERT_FALSE(ctrl.startAllGreenDiagnostics(4000, 10000, &rejReason));
    TEST_ASSERT_NOT_NULL(rejReason);
    TEST_ASSERT_FALSE(ctrl.isAllGreenTestActive());
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());

    // 2. Unlock explicit isolated bench mode
    ctrl.unlockBenchDiagnostics(true);
    TEST_ASSERT_TRUE(ctrl.isBenchDiagnosticsUnlocked());

    // Initiate controlled 10-second ALL-GREEN test with explicit bench unlock
    const bool started = ctrl.startAllGreenDiagnostics(5000, 10000, &rejReason);
    TEST_ASSERT_TRUE(started);
    TEST_ASSERT_TRUE(ctrl.isAllGreenTestActive());
    TEST_ASSERT_EQUAL(TrafficState::ALL_GREEN_DIAG, ctrl.getState());

    // Verify simultaneous vehicle greens and red/yellow/pedestrian walk suppression
    const SignalOutputs diagOut = ctrl.getOutputs();
    TEST_ASSERT_TRUE(diagOut.mainGreen);
    TEST_ASSERT_TRUE(diagOut.sideGreen);
    TEST_ASSERT_TRUE(diagOut.turnGreen);
    TEST_ASSERT_FALSE(diagOut.mainYellow);
    TEST_ASSERT_FALSE(diagOut.sideYellow);
    TEST_ASSERT_FALSE(diagOut.turnYellow);
    TEST_ASSERT_FALSE(diagOut.mainRed);
    TEST_ASSERT_FALSE(diagOut.sideRed);
    TEST_ASSERT_FALSE(diagOut.turnRed);
    TEST_ASSERT_FALSE(diagOut.pedWalk);
    TEST_ASSERT_FALSE(diagOut.ped2Walk);
    TEST_ASSERT_TRUE(diagOut.pedDontWalk);
    TEST_ASSERT_TRUE(diagOut.ped2DontWalk);

    // Verify diagnostic safety invariants explicitly pass for ALL_GREEN_DIAG
    const char* invReason = nullptr;
    TEST_ASSERT_TRUE(TrafficController::verifySafetyInvariants(TrafficState::ALL_GREEN_DIAG, diagOut, &invReason));

    // Mid-test non-blocking execution (5 seconds elapsed)
    ctrl.update(10000);
    TEST_ASSERT_TRUE(ctrl.isAllGreenTestActive());
    TEST_ASSERT_EQUAL(TrafficState::ALL_GREEN_DIAG, ctrl.getState());
    TEST_ASSERT_EQUAL_UINT32(5000, ctrl.getAllGreenRemainingMs(10000));

    // Exact 10.0-second completion boundary
    ctrl.update(15000);
    TEST_ASSERT_FALSE(ctrl.isAllGreenTestActive());
    TEST_ASSERT_TRUE(ctrl.getAllGreenDiagnosticSession().completed);
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());
    TEST_ASSERT_EQUAL(ControlMode::MANUAL, ctrl.getControlMode());

    // Verify post-test state is strictly ALL_RED hold
    const SignalOutputs holdOut = ctrl.getOutputs();
    TEST_ASSERT_TRUE(holdOut.mainRed);
    TEST_ASSERT_TRUE(holdOut.sideRed);
    TEST_ASSERT_TRUE(holdOut.turnRed);
    TEST_ASSERT_FALSE(holdOut.mainGreen);
    TEST_ASSERT_FALSE(holdOut.sideGreen);
    TEST_ASSERT_FALSE(holdOut.turnGreen);

    // Verify holds safely without operator intervention
    ctrl.update(45000);
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());
    TEST_ASSERT_EQUAL(ControlMode::MANUAL, ctrl.getControlMode());
}

void test_all_green_diagnostic_abort_recovery(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.commissionVehicleDriver(50, true);
    ctrl.unlockBenchDiagnostics(true);
    ctrl.update(1000);
    ctrl.update(3000);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());

    TEST_ASSERT_TRUE(ctrl.startAllGreenDiagnostics(5000, 10000));
    TEST_ASSERT_TRUE(ctrl.isAllGreenTestActive());

    // Operator aborts after 2 seconds
    ctrl.cancelAllGreenDiagnostics(7000, "MANUAL_BENCH_ABORT");
    TEST_ASSERT_FALSE(ctrl.isAllGreenTestActive());
    TEST_ASSERT_TRUE(ctrl.getAllGreenDiagnosticSession().aborted);
    TEST_ASSERT_EQUAL_STRING("MANUAL_BENCH_ABORT", ctrl.getAllGreenDiagnosticSession().abortReason);

    // Controller transitions immediately to safe ALL_RED hold
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());
    TEST_ASSERT_EQUAL(ControlMode::MANUAL, ctrl.getControlMode());
    const SignalOutputs out = ctrl.getOutputs();
    TEST_ASSERT_TRUE(out.mainRed && out.sideRed && out.turnRed);
    TEST_ASSERT_FALSE(out.mainGreen || out.sideGreen || out.turnGreen);
}

void test_all_green_conflicting_greens_prevented_in_normal_states(void) {
    TrafficController ctrl;
    ctrl.begin(1000);

    // 1. Verify conflicting greens are strictly prohibited during normal phases
    SignalOutputs illegalOut = TrafficController::deriveOutputsForState(TrafficState::MAIN_GREEN);
    illegalOut.sideGreen = true; // Inject illegal concurrent side green
    const char* reason = nullptr;
    TEST_ASSERT_FALSE(TrafficController::verifySafetyInvariants(TrafficState::MAIN_GREEN, illegalOut, &reason));
    TEST_ASSERT_EQUAL_STRING("CONFLICTING_VEHICLE_GREENS", reason);

    // 2. Verify ALL_GREEN test is rejected by default in normal operational / commissioning mode
    const char* lockedReason = nullptr;
    TEST_ASSERT_FALSE(ctrl.startAllGreenDiagnostics(1500, 10000, &lockedReason));
    TEST_ASSERT_NOT_NULL(lockedReason);

    // 3. Verify ALL_GREEN test is rejected if system is in FAULT_SAFE even if unlocked
    ctrl.unlockBenchDiagnostics(true);
    ctrl.triggerFault(2000, "OPERATOR_INJECTED_FAULT");
    TEST_ASSERT_EQUAL(TrafficState::FAULT_SAFE, ctrl.getState());
    const char* rejReason = nullptr;
    TEST_ASSERT_FALSE(ctrl.startAllGreenDiagnostics(3000, 10000, &rejReason));
    TEST_ASSERT_NOT_NULL(rejReason);
}

// ============================================================================
// PART IV: INDEPENDENT SENSOR OPERATION & 14-CONDITION VALIDATION
// ============================================================================

// 1. All six sensors CLEAR -> all permitted movements (MAIN, SIDE, TURN) receive scheduled green
void test_req1_all_six_sensors_clear_all_movements_receive_scheduled_green(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.setControlMode(0, ControlMode::FIXED);
    ctrl.setProtectedTurnCommissioned(0, true);

    // All six sensors are physically CLEAR (idle HIGH, active-LOW sensors)
    bool irClear[6] = { true, true, true, true, true, true };
    ctrl.processAllPhysicalInputs(500, irClear, true, true);

    ctrl.update(1000); // -> ALL_RED
    ctrl.update(2000); // -> MAIN_GREEN
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());
    TEST_ASSERT_TRUE(ctrl.getOutputs().mainGreen);

    // Advance to SIDE_GREEN
    ctrl.update(6000); // -> MAIN_YELLOW
    ctrl.update(8000); // -> ALL_RED
    ctrl.update(9000); // -> SIDE_GREEN
    TEST_ASSERT_EQUAL(TrafficState::SIDE_GREEN, ctrl.getState());
    TEST_ASSERT_TRUE(ctrl.getOutputs().sideGreen);

    // Advance to TURN_GREEN
    ctrl.update(11000); // -> SIDE_YELLOW
    ctrl.update(13000); // -> ALL_RED
    ctrl.update(14000); // -> TURN_GREEN
    TEST_ASSERT_EQUAL(TrafficState::TURN_GREEN, ctrl.getState());
    TEST_ASSERT_TRUE(ctrl.getOutputs().turnGreen);

    // Advance back to MAIN_GREEN
    ctrl.update(16000); // -> TURN_YELLOW
    ctrl.update(18000); // -> ALL_RED
    ctrl.update(19000); // -> MAIN_GREEN
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());
    TEST_ASSERT_TRUE(ctrl.getOutputs().mainGreen);
}

// 2. MAIN sensors both CLEAR -> MAIN still receives scheduled green
void test_req2_main_sensors_both_clear_main_receives_scheduled_green(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.setControlMode(0, ControlMode::FIXED);
    ctrl.setProtectedTurnCommissioned(0, true);

    // Main S1 (GPIO 33) and Main S2 (GPIO 34) CLEAR; other sensors may have vehicles
    bool irPins[6] = { true, true, false, true, false, true };
    ctrl.processAllPhysicalInputs(500, irPins, true, true);

    ctrl.update(1000); // -> ALL_RED
    ctrl.update(2000); // -> MAIN_GREEN
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());
    TEST_ASSERT_TRUE(ctrl.getOutputs().mainGreen);
    TEST_ASSERT_FALSE(ctrl.hasMainVehicleDemand());
}

// 3. SIDE sensors both CLEAR -> SIDE still receives scheduled green
void test_req3_side_sensors_both_clear_side_receives_scheduled_green(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.setControlMode(0, ControlMode::FIXED);
    ctrl.setProtectedTurnCommissioned(0, true);

    // Side S1 (GPIO 35) and Side S2 (GPIO 36) CLEAR
    bool irPins[6] = { true, true, true, true, false, true };
    ctrl.processAllPhysicalInputs(500, irPins, true, true);

    ctrl.update(1000); // -> ALL_RED
    ctrl.update(2000); // -> MAIN_GREEN
    ctrl.update(6000); // -> MAIN_YELLOW
    ctrl.update(8000); // -> ALL_RED
    ctrl.update(9000); // -> SIDE_GREEN
    TEST_ASSERT_EQUAL(TrafficState::SIDE_GREEN, ctrl.getState());
    TEST_ASSERT_TRUE(ctrl.getOutputs().sideGreen);
    TEST_ASSERT_FALSE(ctrl.hasSideVehicleDemand());
}

// 4. TURN sensors both CLEAR -> TURN receives green during permitted phase
void test_req4_turn_sensors_both_clear_turn_receives_scheduled_green(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.setControlMode(0, ControlMode::FIXED);
    ctrl.setProtectedTurnCommissioned(0, true);

    // Turn S1 (GPIO 39) and Turn S2 (GPIO 15) CLEAR
    bool irPins[6] = { true, true, true, true, true, true };
    ctrl.processAllPhysicalInputs(500, irPins, true, true);

    ctrl.update(1000); // -> ALL_RED
    ctrl.update(2000); // -> MAIN_GREEN
    ctrl.update(6000); // -> MAIN_YELLOW
    ctrl.update(8000); // -> ALL_RED
    ctrl.update(9000); // -> SIDE_GREEN
    ctrl.update(11000); // -> SIDE_YELLOW
    ctrl.update(13000); // -> ALL_RED
    ctrl.update(14000); // -> TURN_GREEN
    TEST_ASSERT_EQUAL(TrafficState::TURN_GREEN, ctrl.getState());
    TEST_ASSERT_TRUE(ctrl.getOutputs().turnGreen);
    TEST_ASSERT_FALSE(ctrl.hasTurnVehicleDemand());
}

// 5. One sensor detects -> demand registered immediately without waiting for 2nd
void test_req5_one_sensor_detects_demand_registered(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.setPhysicalIrValidated(true);

    // Only Side S1 (IR3 / GPIO 35) detects obstacle (LOW); Side S2 (IR4 / GPIO 36) remains CLEAR (HIGH)
    bool irPins[6] = { true, true, false, true, true, true };
    ctrl.processAllPhysicalInputs(100, irPins, true, true);
    ctrl.processAllPhysicalInputs(250, irPins, true, true); // > 100ms debounce
    TEST_ASSERT_TRUE(ctrl.hasSideVehicleDemand());
    TEST_ASSERT_TRUE(ctrl.getVehicleDetector().isRoadSensorDetected(TrafficConfig::MovementId::SIDE_ROAD, 0));
    TEST_ASSERT_FALSE(ctrl.getVehicleDetector().isRoadSensorDetected(TrafficConfig::MovementId::SIDE_ROAD, 1));
}

// 6. Both sensors detect -> approach demand registered without double-counting
void test_req6_both_sensors_detect_no_double_counting(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.setPhysicalIrValidated(true);

    // Both Side S1 (GPIO 35) and Side S2 (GPIO 36) detect obstacle (LOW)
    bool irPins[6] = { true, true, false, false, true, true };
    ctrl.processAllPhysicalInputs(100, irPins, true, true);
    ctrl.processAllPhysicalInputs(250, irPins, true, true);

    TEST_ASSERT_TRUE(ctrl.hasSideVehicleDemand());
    TEST_ASSERT_TRUE(ctrl.getVehicleDetector().isRoadSensorDetected(TrafficConfig::MovementId::SIDE_ROAD, 0));
    TEST_ASSERT_TRUE(ctrl.getVehicleDetector().isRoadSensorDetected(TrafficConfig::MovementId::SIDE_ROAD, 1));
    // Approach demand remains a single boolean demand, not duplicated
    const ApproachDemandState& st = ctrl.getDemandManager().getApproachState(TrafficConfig::MovementId::SIDE_ROAD);
    TEST_ASSERT_TRUE(st.hasDemand);
}

// 7. One sensor fails -> fixed-time fallback operates
void test_req7_one_sensor_fails_fixed_time_fallback_operates(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.setPhysicalIrValidated(true);
    ctrl.setControlMode(0, ControlMode::ADAPTIVE);
    ctrl.setProtectedTurnCommissioned(0, true);

    ctrl.update(1000); // -> ALL_RED
    ctrl.update(2000); // -> MAIN_GREEN

    // Inject chatter fault on Side S1 (IR3 / GPIO 35)
    for (uint8_t i = 0; i < 15; ++i) {
        bool irPins[6] = { true, true, (i % 2 == 0), true, true, true };
        ctrl.processAllPhysicalInputs(2100 + i * 20, irPins, true, true);
    }
    TEST_ASSERT_TRUE(ctrl.getVehicleDetector().isAnyChannelFaulted());

    // Adaptive mode falls back to fixed-time rotation and serves SIDE_GREEN!
    ctrl.update(6000); // -> MAIN_YELLOW
    ctrl.update(8000); // -> ALL_RED
    ctrl.update(9000); // -> SIDE_GREEN (served despite sensor fault!)
    TEST_ASSERT_EQUAL(TrafficState::SIDE_GREEN, ctrl.getState());
}

// 8. All sensors disconnected -> fixed-time operation continues
void test_req8_all_sensors_disconnected_fixed_time_continues(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.setControlMode(0, ControlMode::FIXED);
    ctrl.setProtectedTurnCommissioned(0, true);

    // Disconnected sensors float HIGH (idle)
    bool irFloating[6] = { true, true, true, true, true, true };
    ctrl.processAllPhysicalInputs(500, irFloating, true, true);

    // Verify continuous rotation without interruption
    ctrl.update(1000); // ALL_RED
    ctrl.update(2000); // MAIN_GREEN
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());
    ctrl.update(6000); // MAIN_YELLOW
    ctrl.update(8000); // ALL_RED
    ctrl.update(9000); // SIDE_GREEN
    TEST_ASSERT_EQUAL(TrafficState::SIDE_GREEN, ctrl.getState());
    ctrl.update(11000); // SIDE_YELLOW
    ctrl.update(13000); // ALL_RED
    ctrl.update(14000); // TURN_GREEN
    TEST_ASSERT_EQUAL(TrafficState::TURN_GREEN, ctrl.getState());
}

// 9. All sensors unvalidated -> fixed-time operation continues
void test_req9_all_sensors_unvalidated_fixed_time_continues(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.setPhysicalIrValidated(false); // Unvalidate all sensors
    ctrl.setControlMode(0, ControlMode::FIXED);
    ctrl.setProtectedTurnCommissioned(0, true);

    TEST_ASSERT_FALSE(ctrl.isPhysicalIrValidated());

    ctrl.update(1000);
    ctrl.update(2000);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());
    ctrl.update(6000);
    ctrl.update(8000);
    ctrl.update(9000);
    TEST_ASSERT_EQUAL(TrafficState::SIDE_GREEN, ctrl.getState());
    ctrl.update(11000);
    ctrl.update(13000);
    ctrl.update(14000);
    TEST_ASSERT_EQUAL(TrafficState::TURN_GREEN, ctrl.getState());
}

// 10. Sensor data becomes stale -> fixed-time fallback operates
void test_req10_sensor_data_stale_fixed_time_fallback_operates(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.setPhysicalIrValidated(true);
    ctrl.setControlMode(0, ControlMode::ADAPTIVE);
    ctrl.setProtectedTurnCommissioned(0, true);

    ctrl.update(1000); // ALL_RED
    ctrl.update(2000); // MAIN_GREEN

    // Invalidate hardware adapter frames (stale / broken stream)
    HardwareInputs invalidFrame = { false, false, false };
    ctrl.processHardwareAdapter(2500, invalidFrame);
    ctrl.processHardwareAdapter(3000, invalidFrame);

    // Controller falls back to fixed-time rotation
    ctrl.update(6000); // MAIN_YELLOW
    ctrl.update(8000); // ALL_RED
    ctrl.update(9000); // SIDE_GREEN
    TEST_ASSERT_EQUAL(TrafficState::SIDE_GREEN, ctrl.getState());
}

// 11. No movement is starved indefinitely (max green bounding)
void test_req11_no_movement_starved_indefinitely(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.setPhysicalIrValidated(true);
    ctrl.setControlMode(0, ControlMode::ACTUATED);
    ctrl.setProtectedTurnCommissioned(0, true);

    ctrl.update(1000);
    ctrl.update(2000); // MAIN_GREEN

    // Main road has continuous vehicle presence, but Side road also has waiting demand
    bool irPins[6] = { false, true, false, true, true, true }; // Main S1 LOW, Side S1 LOW
    ctrl.processAllPhysicalInputs(2100, irPins, true, true);
    ctrl.processAllPhysicalInputs(2250, irPins, true, true);

    // Main green extends past min green (4000 ms -> t=6000)
    ctrl.update(6000);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());

    // Main road cannot hold green past max green (12000 ms -> t=14000)
    ctrl.update(14000);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_YELLOW, ctrl.getState());
    ctrl.update(16000); // ALL_RED
    ctrl.update(17000); // SIDE_GREEN is served! (no starvation)
    TEST_ASSERT_EQUAL(TrafficState::SIDE_GREEN, ctrl.getState());
}

// 12. No conflicting vehicle greens occur
void test_req12_no_conflicting_vehicle_greens(void) {
    TrafficController ctrl;
    ctrl.begin(0);

    // Test that verifySafetyInvariants strictly catches any simultaneous vehicle green
    SignalOutputs s = TrafficController::deriveOutputsForState(TrafficState::MAIN_GREEN);
    s.sideGreen = true;
    const char* reason = nullptr;
    TEST_ASSERT_FALSE(TrafficController::verifySafetyInvariants(TrafficState::MAIN_GREEN, s, &reason));
    TEST_ASSERT_EQUAL_STRING("CONFLICTING_VEHICLE_GREENS", reason);

    s = TrafficController::deriveOutputsForState(TrafficState::MAIN_GREEN);
    s.turnGreen = true;
    TEST_ASSERT_FALSE(TrafficController::verifySafetyInvariants(TrafficState::MAIN_GREEN, s, &reason));

    s = TrafficController::deriveOutputsForState(TrafficState::SIDE_GREEN);
    s.turnGreen = true;
    TEST_ASSERT_FALSE(TrafficController::verifySafetyInvariants(TrafficState::SIDE_GREEN, s, &reason));
}

// 13. Yellow and all-red clearance remain strictly enforced
void test_req13_yellow_and_all_red_clearance_enforced(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.setControlMode(0, ControlMode::FIXED);
    ctrl.setProtectedTurnCommissioned(0, true);

    ctrl.update(1000); // ALL_RED
    ctrl.update(2000); // MAIN_GREEN

    // MAIN_GREEN transitions to MAIN_YELLOW first (never directly to RED or SIDE_GREEN)
    ctrl.update(6000);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_YELLOW, ctrl.getState());
    TEST_ASSERT_TRUE(ctrl.getOutputs().mainYellow);
    TEST_ASSERT_FALSE(ctrl.getOutputs().sideGreen);

    // MAIN_YELLOW transitions to ALL_RED
    ctrl.update(8000);
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());
    TEST_ASSERT_TRUE(ctrl.getOutputs().mainRed && ctrl.getOutputs().sideRed && ctrl.getOutputs().turnRed);

    // Then SIDE_GREEN
    ctrl.update(9000);
    TEST_ASSERT_EQUAL(TrafficState::SIDE_GREEN, ctrl.getState());
}

// 14. Pedestrian requests served safely without conflicting vehicle greens
void test_req14_pedestrian_requests_served_safely(void) {
    TrafficController ctrl;
    ctrl.begin(0);
    ctrl.setSimulationMode(0, true);

    ctrl.update(1000); // ALL_RED
    ctrl.update(2000); // MAIN_GREEN

    // Pedestrian requests arrive for both crossings
    ctrl.triggerSimPedestrianChannel(2500, 0);
    ctrl.triggerSimPedestrianChannel(2500, 1);
    TEST_ASSERT_TRUE(ctrl.hasPedestrianDemand());

    ctrl.update(6000); // MAIN_YELLOW
    ctrl.update(8000); // ALL_RED
    ctrl.update(9000); // PED_WALK
    TEST_ASSERT_EQUAL(TrafficState::PED_WALK, ctrl.getState());

    // During PED_WALK, pedestrian WALK is active, and ALL vehicle signals are RED
    SignalOutputs out = ctrl.getOutputs();
    TEST_ASSERT_TRUE(out.pedWalk);
    TEST_ASSERT_TRUE(out.ped2Walk);
    TEST_ASSERT_TRUE(out.mainRed);
    TEST_ASSERT_TRUE(out.sideRed);
    TEST_ASSERT_TRUE(out.turnRed);
    TEST_ASSERT_FALSE(out.mainGreen);
    TEST_ASSERT_FALSE(out.sideGreen);
    TEST_ASSERT_FALSE(out.turnGreen);
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

    // Extended v2.0.0 & RHT alignment comprehensive tests (14..39)
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
    RUN_TEST(test_serial_timing_adjustment_atomic_and_nvs_persistence);
    RUN_TEST(test_every_safety_invariant_13_channels_and_emergency_guard);
    RUN_TEST(test_locked_gpio_assignments_and_all_13_signal_outputs);
    RUN_TEST(test_all_red_physical_red_and_green_lamp_mapping);
    RUN_TEST(test_two_independent_sensors_per_road_architecture);
    RUN_TEST(test_four_layer_signal_output_driver_abstraction);
    RUN_TEST(test_rht_configuration_and_movement_definitions);
    RUN_TEST(test_rht_protected_turn_to_side_and_turn_to_main_transitions);
    RUN_TEST(test_rht_conflict_matrix_and_pedestrian_crossing_relationships);
    RUN_TEST(test_dashboard_status_snapshot_and_13_channel_telemetry_parsing);

    // Phase 1-8 Operational Activation & Commissioning tests (40..45)
    RUN_TEST(test_runtime_driver_commissioning_and_common_ground_interlock);
    RUN_TEST(test_single_lamp_test_mode_safety_and_interlocks);
    RUN_TEST(test_ir_sensor_release_detection_and_per_channel_commissioning);
    RUN_TEST(test_adaptive_pedestrian_waiting_priority_and_starvation_boost);
    RUN_TEST(test_manual_mode_clearance_step_rejection);
    RUN_TEST(test_nvs_first_boot_initialization_and_commissioning_persistence);
    RUN_TEST(test_confirmed_direct_vehicle_and_uln2003a_pedestrian_polarity);
    RUN_TEST(test_comprehensive_fault_architecture_and_recovery_sequencing);
    RUN_TEST(test_fixed_time_prototype_sequence_and_sensor_isolation);
    RUN_TEST(test_adaptive_mode_sensor_fault_fallback_recall);
    RUN_TEST(test_all_green_diagnostic_10s_execution_and_all_red_hold);
    RUN_TEST(test_all_green_diagnostic_abort_recovery);
    RUN_TEST(test_all_green_conflicting_greens_prevented_in_normal_states);

    // Part IV: Independent sensor operation & 14-condition validation
    RUN_TEST(test_req1_all_six_sensors_clear_all_movements_receive_scheduled_green);
    RUN_TEST(test_req2_main_sensors_both_clear_main_receives_scheduled_green);
    RUN_TEST(test_req3_side_sensors_both_clear_side_receives_scheduled_green);
    RUN_TEST(test_req4_turn_sensors_both_clear_turn_receives_scheduled_green);
    RUN_TEST(test_req5_one_sensor_detects_demand_registered);
    RUN_TEST(test_req6_both_sensors_detect_no_double_counting);
    RUN_TEST(test_req7_one_sensor_fails_fixed_time_fallback_operates);
    RUN_TEST(test_req8_all_sensors_disconnected_fixed_time_continues);
    RUN_TEST(test_req9_all_sensors_unvalidated_fixed_time_continues);
    RUN_TEST(test_req10_sensor_data_stale_fixed_time_fallback_operates);
    RUN_TEST(test_req11_no_movement_starved_indefinitely);
    RUN_TEST(test_req12_no_conflicting_vehicle_greens);
    RUN_TEST(test_req13_yellow_and_all_red_clearance_enforced);
    RUN_TEST(test_req14_pedestrian_requests_served_safely);
    UNITY_END();
}

void loop() {}
