#include <Arduino.h>
#include <unity.h>
#include "TrafficConfig.h"
#include "TrafficController.h"

void setUp(void) {}
void tearDown(void) {}

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

    // Before STARTUP_SAFE_MS (1000 ms)
    ctrl.update(999);
    TEST_ASSERT_EQUAL(TrafficState::STARTUP_SAFE, ctrl.getState());

    // At 1000 ms -> ALL_RED
    ctrl.update(1000);
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());
    SignalOutputs outAllRed = ctrl.getOutputs();
    TEST_ASSERT_TRUE(outAllRed.mainRed);
    TEST_ASSERT_TRUE(outAllRed.sideRed);
    TEST_ASSERT_TRUE(outAllRed.pedDontWalk);
    TEST_ASSERT_FALSE(outAllRed.mainGreen);
    TEST_ASSERT_FALSE(outAllRed.sideGreen);
    TEST_ASSERT_FALSE(outAllRed.pedWalk);

    // Before ALL_RED_MS (1000 ms elapsed from 1000 -> 1999)
    ctrl.update(1999);
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());

    // At 2000 ms -> MAIN_GREEN
    ctrl.update(2000);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());
    SignalOutputs outMain = ctrl.getOutputs();
    TEST_ASSERT_TRUE(outMain.mainGreen);
    TEST_ASSERT_TRUE(outMain.sideRed);
    TEST_ASSERT_TRUE(outMain.pedDontWalk);
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
    TEST_ASSERT_FALSE(outWalk.mainGreen);
    TEST_ASSERT_FALSE(outWalk.sideGreen);

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
    // Default mode is HARDWARE_READY
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
        TEST_ASSERT_FALSE(outReset.pedWalk);
    }
}

// 8. Test millis() rollover-safe timing across 0xFFFFFFFF -> 0x00000000 wraparound
void test_millis_rollover_safety(void) {
    TrafficController ctrl;
    const uint32_t startNearWrap = 0xFFFFFFFFUL - 400UL; // 400 ms before 32-bit rollover
    ctrl.begin(startNearWrap);
    TEST_ASSERT_EQUAL(TrafficState::STARTUP_SAFE, ctrl.getState());

    // Advance 300 ms (still before rollover)
    ctrl.update(startNearWrap + 300UL);
    TEST_ASSERT_EQUAL(TrafficState::STARTUP_SAFE, ctrl.getState());

    // Advance across rollover to 999 ms elapsed
    ctrl.update(startNearWrap + 999UL);
    TEST_ASSERT_EQUAL(TrafficState::STARTUP_SAFE, ctrl.getState());

    // Advance to 1000 ms elapsed (wrapped around 0) -> ALL_RED
    ctrl.update(startNearWrap + 1000UL);
    TEST_ASSERT_EQUAL(TrafficState::ALL_RED, ctrl.getState());

    // Advance another 1000 ms -> MAIN_GREEN
    ctrl.update(startNearWrap + 2000UL);
    TEST_ASSERT_EQUAL(TrafficState::MAIN_GREEN, ctrl.getState());
}

// 9. Test Prevention of conflicting greens, vehicle green during WALK, and Fault-state output behavior
void test_safety_invariants_and_fault_safe(void) {
    // Verify all valid states satisfy invariants
    const TrafficState allStates[] = {
        TrafficState::STARTUP_SAFE,
        TrafficState::ALL_RED,
        TrafficState::MAIN_GREEN,
        TrafficState::MAIN_YELLOW,
        TrafficState::SIDE_GREEN,
        TrafficState::SIDE_YELLOW,
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
    TEST_ASSERT_FALSE(faultOut.pedWalk);
    TEST_ASSERT_FALSE(faultOut.pedDontWalk);

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

    // Initial unpressed state (HIGH on INPUT_PULLUP)
    ctrl.processPhysicalInputs(2000, true, true);
    TEST_ASSERT_FALSE(ctrl.hasPedestrianDemand());

    // Noisy glitch (< 50 ms LOW)
    ctrl.processPhysicalInputs(2010, false, true);
    ctrl.processPhysicalInputs(2040, false, true);
    ctrl.processPhysicalInputs(2045, true, true); // Bounces back HIGH before 50 ms
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

    // Advance across multiple minutes with no demand
    for (uint32_t t = 3000; t <= 120000; t += 1000) {
        HardwareInputs idleInputs = {true, true, true}; // Pull-ups HIGH (inactive)
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

    // Attempt to set invalid timings (0 ms clearance/yellow, maxGreen < minGreen, excessive values)
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

    // setTimingConfig must return false (indicating invalid values were sanitized)
    TEST_ASSERT_FALSE(ctrl.setTimingConfig(0, badCfg));

    // Verify all invalid fields fell back to documented safe defaults
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

    // Case A: Physical IR sensor is unvalidated by default (PHYSICAL_IR_VALIDATED == false).
    // Even if GPIO 33 reads LOW continuously, it must NOT trigger side-road demand.
    TEST_ASSERT_FALSE(ctrl.isPhysicalIrValidated());
    ctrl.processPhysicalInputs(2100, true, false);
    ctrl.processPhysicalInputs(2500, true, false);
    TEST_ASSERT_FALSE(ctrl.hasSideVehicleDemand());

    // Case B: Invalid hardware adapter frame (inputsValid == false) is ignored safely
    ctrl.setPhysicalIrValidated(true);
    HardwareInputs invalidFrame = {false, false, false};
    ctrl.processHardwareAdapter(2600, invalidFrame);
    ctrl.processHardwareAdapter(2800, invalidFrame);
    TEST_ASSERT_FALSE(ctrl.hasSideVehicleDemand());
    TEST_ASSERT_FALSE(ctrl.hasPedestrianDemand());

    // Case C: Validated sensor shorted/stuck active LOW beyond MAX_INPUT_STUCK_MS (30000 ms)
    // is flagged as suspected fault and suppressed so it cannot monopolize the intersection.
    ctrl.processPhysicalInputs(3000, true, false);
    ctrl.processPhysicalInputs(3200, true, false); // Stable LOW -> demand registered
    TEST_ASSERT_TRUE(ctrl.hasSideVehicleDemand());
    TEST_ASSERT_FALSE(ctrl.isHwIrFaultSuspected());

    // Sensor remains stuck LOW for > 30000 ms
    ctrl.processPhysicalInputs(33300, true, false);
    TEST_ASSERT_TRUE(ctrl.isHwIrFaultSuspected());
    TEST_ASSERT_FALSE(ctrl.hasSideVehicleDemand()); // Demand suppressed while stuck!

    // Once physical sensor returns HIGH stably, stuck fault clears automatically
    ctrl.processPhysicalInputs(34000, true, true);
    ctrl.processPhysicalInputs(34200, true, true);
    TEST_ASSERT_FALSE(ctrl.isHwIrFaultSuspected());
}

void setup() {
    delay(2000);
    UNITY_BEGIN();
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
    UNITY_END();
}

void loop() {}

