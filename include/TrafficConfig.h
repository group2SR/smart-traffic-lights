#ifndef TRAFFIC_CONFIG_H
#define TRAFFIC_CONFIG_H

#include <stdint.h>
#include <stddef.h>

namespace TrafficConfig {

// ============================================================================
// PROJECT METADATA
// ============================================================================
constexpr const char* PROJECT_NAME    = "ESP32 Smart Traffic Light Controller (3-Way T-Junction)";
constexpr const char* FIRMWARE_VER    = "2.0.0-full-adaptive-control";
constexpr uint32_t    SERIAL_BAUD     = 115200;

// Traffic Rule Specification:
// This project uses RIGHT-HAND TRAFFIC (KEEP-RIGHT).
// Physical junction turning paths and protected-turn geometry must be explicitly
// verified before commissioning protected-turn phases on physical lamps.
constexpr bool RIGHT_HAND_TRAFFIC_RULE = true;

// ============================================================================
// MOVEMENT & CHANNEL COUNTS
// ============================================================================
constexpr uint8_t VEHICLE_GROUP_COUNT     = 3; // Main, Side, Protected Turn
constexpr uint8_t PED_GROUP_COUNT         = 2; // Pedestrian 1, Pedestrian 2
constexpr uint8_t IR_SENSOR_COUNT         = 6; // IR1 .. IR6
constexpr uint8_t PED_BUTTON_COUNT        = 2; // PED1, PED2
constexpr uint8_t TOTAL_SIGNAL_OUTPUTS    = 13; // 9 vehicle + 4 pedestrian

enum class MovementId : uint8_t {
    MAIN_ROAD      = 0,
    SIDE_ROAD      = 1,
    PROTECTED_TURN = 2,
    PED_CROSSING_1 = 3,
    PED_CROSSING_2 = 4,
    COUNT          = 5
};

// ============================================================================
// CENTRALIZED GPIO PIN ASSIGNMENTS (38-Pin ESP32-WROOM-32 DevKit V1)
// ============================================================================
// Existing wired pins from v1.1.0 are preserved by default so existing wiring
// is not silently altered.
//
// IMPORTANT HARDWARE NOTE ON GPIO 12 (PIN_SIDE_YELLOW):
// GPIO 12 is an ESP32 boot-strapping pin (MTDI) that selects VDD_SDIO flash
// voltage at reset (LOW = 3.3V flash, HIGH = 1.8V flash).
// - PIN_SIDE_YELLOW (12) is retained as the legacy constant so existing wiring
//   is not silently broken.
// - PIN_SIDE_YELLOW_SAFE_PROPOSED (5) is provided as the recommended non-MTDI
//   re-wire target (requires moving 1 physical wire from GPIO 12 to GPIO 5).

// Vehicle Signal Group 1: Main Road (RYG)
constexpr uint8_t PIN_MAIN_GREEN                  = 27; // OUTPUT (Safe GP GPIO)
constexpr uint8_t PIN_MAIN_YELLOW                 = 26; // OUTPUT (Safe GP GPIO)
constexpr uint8_t PIN_MAIN_RED                    = 25; // OUTPUT (Safe GP GPIO)

// Vehicle Signal Group 2: Side Road (RYG)
constexpr uint8_t PIN_SIDE_GREEN                  = 13; // OUTPUT (Safe GP GPIO)
constexpr uint8_t PIN_SIDE_YELLOW                 = 12; // OUTPUT (Legacy v1.1.0 pin; MTDI boot-strap risk)
constexpr uint8_t PIN_SIDE_YELLOW_SAFE_PROPOSED   = 5;  // OUTPUT (Proposed safe pin after physical rewiring)
constexpr uint8_t PIN_SIDE_RED                    = 14; // OUTPUT (Safe GP GPIO)

// Vehicle Signal Group 3: Protected Turn (RYG)
constexpr uint8_t PIN_TURN_GREEN                  = 23; // OUTPUT (Safe GP GPIO)
constexpr uint8_t PIN_TURN_YELLOW                 = 22; // OUTPUT (Safe GP GPIO)
constexpr uint8_t PIN_TURN_RED                    = 21; // OUTPUT (Safe GP GPIO)

// Pedestrian Signal Group 1 (RED = Don't Walk, GREEN = Walk)
constexpr uint8_t PIN_PED1_WALK                   = 19; // OUTPUT (Safe GP GPIO)
constexpr uint8_t PIN_PED1_DONT_WALK              = 18; // OUTPUT (Safe GP GPIO)
// Legacy aliases preserved for backward compatibility
constexpr uint8_t PIN_PED_WALK                    = PIN_PED1_WALK;
constexpr uint8_t PIN_PED_DONT_WALK               = PIN_PED1_DONT_WALK;

// Pedestrian Signal Group 2 (RED = Don't Walk, GREEN = Walk)
constexpr uint8_t PIN_PED2_WALK                   = 17; // OUTPUT (Safe GP GPIO)
constexpr uint8_t PIN_PED2_DONT_WALK              = 16; // OUTPUT (Safe GP GPIO)

// Pedestrian Pushbutton Inputs (Active LOW with INPUT_PULLUP)
constexpr uint8_t PIN_PED1_BUTTON                 = 32; // INPUT_PULLUP (Safe GP GPIO)
constexpr uint8_t PIN_PED2_BUTTON                 = 4;  // INPUT_PULLUP (Safe GP GPIO, non-strapping)
// Legacy alias preserved
constexpr uint8_t PIN_PED_BUTTON                  = PIN_PED1_BUTTON;

// Six Independent IR Vehicle Detection Inputs (Active LOW)
// Note: GPIO 34, 35, 36, 39 are input-only and lack internal pull-up resistors;
// they require external 10k-ohm pull-up resistors to 3.3V and 3.3V level clamping.
constexpr uint8_t PIN_IR_SENSOR_1                 = 33; // INPUT_PULLUP (Side Road Primary / Legacy PIN_SIDE_IR_SENSOR)
constexpr uint8_t PIN_IR_SENSOR_2                 = 34; // INPUT (Input-Only: Side Road Secondary / Queue)
constexpr uint8_t PIN_IR_SENSOR_3                 = 35; // INPUT (Input-Only: Main Road Primary)
constexpr uint8_t PIN_IR_SENSOR_4                 = 36; // INPUT (Input-Only VP: Main Road Secondary / Queue)
constexpr uint8_t PIN_IR_SENSOR_5                 = 39; // INPUT (Input-Only VN: Protected Turn Primary)
constexpr uint8_t PIN_IR_SENSOR_6                 = 15; // INPUT_PULLUP (Protected Turn Secondary; MTDO idle HIGH)
// Legacy alias preserved
constexpr uint8_t PIN_SIDE_IR_SENSOR              = PIN_IR_SENSOR_1;

constexpr uint8_t IR_SENSOR_PINS[IR_SENSOR_COUNT] = {
    PIN_IR_SENSOR_1,
    PIN_IR_SENSOR_2,
    PIN_IR_SENSOR_3,
    PIN_IR_SENSOR_4,
    PIN_IR_SENSOR_5,
    PIN_IR_SENSOR_6
};

constexpr bool IR_SENSOR_HAS_INTERNAL_PULLUP[IR_SENSOR_COUNT] = {
    true,  // GPIO 33
    false, // GPIO 34 (input-only, external 10k pull-up required)
    false, // GPIO 35 (input-only, external 10k pull-up required)
    false, // GPIO 36 (input-only, external 10k pull-up required)
    false, // GPIO 39 (input-only, external 10k pull-up required)
    true   // GPIO 15
};

// ============================================================================
// ELECTRICAL DRIVER & HARDWARE VALIDATION CONFIGURATION
// ============================================================================
// IMPORTANT ELECTRICAL FINDING:
// - The 3 vehicle RYG modules are 5V COMMON-GROUND (Common-Cathode) modules:
//   applying +5V to R, Y, or G illuminates the lamp.
// - ULN2003A is a low-side current-sinking Darlington array and CANNOT source +5V
//   to common-ground vehicle modules.
// - Therefore, physical vehicle outputs remain gated by VEHICLE_DRIVER_VERIFIED = false
//   until a verified high-side sourcing driver stage is connected.
// - Pedestrian LEDs have individual anodes/cathodes and may be driven by ULN2003A
//   in common-anode (+5V) configuration with series current-limiting resistors.
enum class OutputDriverType : uint8_t {
    UNVERIFIED_SAFE_DISABLED     = 0, // Holds pin at safe electrical off level; blocks physical lamp energization
    DIRECT_LOGIC_ACTIVE_HIGH     = 1, // 3.3V logic active-HIGH (bench logic analyzer / indicator test only)
    HIGH_SIDE_SOURCING_ACTIVE_HIGH = 2, // Non-inverting high-side +5V source driver (GPIO HIGH = +5V ON)
    HIGH_SIDE_SOURCING_ACTIVE_LOW  = 3, // Inverting high-side +5V source driver (GPIO LOW = +5V ON, HIGH = OFF)
    LOW_SIDE_ULN2003A_ACTIVE_HIGH  = 4  // ULN2003A low-side sink with Common-Anode LEDs (GPIO HIGH = sink ON)
};

constexpr bool VEHICLE_HIGH_SIDE_DRIVER_VERIFIED  = false;
constexpr bool PED_ULN2003A_DRIVER_VERIFIED       = false;
constexpr bool PHYSICAL_IR_VALIDATED              = false;
constexpr bool PROTECTED_TURN_GEOMETRY_VERIFIED   = false;

// Input Polarity & Filtering Defaults
constexpr bool     PED_BTN_ACTIVE_LOW    = true;
constexpr bool     IR_SNS_ACTIVE_LOW     = true;
constexpr uint32_t PED_DEBOUNCE_MS       = 50;    // 50 ms pushbutton debounce
constexpr uint32_t IR_STABILITY_MS       = 100;   // 100 ms IR sensor stability filter
constexpr uint32_t MAX_INPUT_STUCK_MS    = 30000; // 30 s stuck-sensor/button fault timeout
constexpr uint32_t IR_CHATTER_WINDOW_MS  = 1000;  // 1 s window for disconnected/noisy chatter detection
constexpr uint8_t  IR_MAX_TRANSITIONS_PER_WINDOW = 12; // >12 transitions/sec indicates floating/chattering input

// ============================================================================
// BENCH-TEST DEMONSTRATION PHASE TIMINGS (MILLISECONDS)
// NOTE: Demonstration values only — NOT certified for public-road use.
// ============================================================================
constexpr uint32_t STARTUP_SAFE_MS       = 1000;  // Startup safe interval before ALL_RED (1 s)
constexpr uint32_t ALL_RED_MS            = 1000;  // All-red clearance interval (1 s)
constexpr uint32_t MAIN_MIN_GREEN_MS     = 4000;  // Minimum main-road green (4 s)
constexpr uint32_t MAIN_MAX_GREEN_MS     = 12000; // Maximum main-road green when conflicting demand waits (12 s)
constexpr uint32_t MAIN_CLEAR_GAP_MS     = 2000;  // Main-road vehicle extension/clear gap (2 s)
constexpr uint32_t MAIN_YELLOW_MS        = 2000;  // Main-road yellow (2 s)

constexpr uint32_t SIDE_MIN_GREEN_MS     = 2000;  // Minimum side-road green (2 s)
constexpr uint32_t SIDE_MAX_GREEN_MS     = 8000;  // Maximum side-road green (8 s)
constexpr uint32_t SIDE_CLEAR_GAP_MS     = 2000;  // Side-road vehicle clear-gap (2 s)
constexpr uint32_t SIDE_YELLOW_MS        = 2000;  // Side-road yellow (2 s)

constexpr uint32_t TURN_MIN_GREEN_MS     = 2000;  // Minimum protected-turn green (2 s)
constexpr uint32_t TURN_MAX_GREEN_MS     = 6000;  // Maximum protected-turn green (6 s)
constexpr uint32_t TURN_CLEAR_GAP_MS     = 2000;  // Protected-turn vehicle clear-gap (2 s)
constexpr uint32_t TURN_YELLOW_MS        = 2000;  // Protected-turn yellow (2 s)

constexpr uint32_t PED_WALK_MS           = 5000;  // Pedestrian WALK interval (5 s)
constexpr uint32_t PED_CLEARANCE_MS      = 1000;  // Post-WALK pedestrian clearance interval (1 s default)

// Safe Fallback Timing Bounds (used when validating custom/runtime TimingConfig)
constexpr uint32_t MIN_ALLOWED_INTERVAL_MS = 500;
constexpr uint32_t MAX_ALLOWED_INTERVAL_MS = 120000;

// Adaptive Demand Scoring Parameters
constexpr uint32_t DEFAULT_WAIT_TIME_WEIGHT_MS   = 2000; // Every 2 s waiting adds +1 demand score
constexpr uint8_t  DEFAULT_ACTIVE_SENSOR_WEIGHT  = 3;    // Each active sensor on approach adds +3 demand score
constexpr uint8_t  DEFAULT_STARVATION_SKIP_LIMIT = 2;    // After 2 skipped cycles, approach receives starvation boost
constexpr uint8_t  DEFAULT_STARVATION_BOOST      = 25;   // Score boost applied when starvation threshold is reached

// Aggregate TimingConfig structure.
// First 9 fields match v1.1.0-hardware-ready in exact order so legacy 9-field
// aggregate initializers remain 100% compatible.
struct TimingConfig {
    // Core v1.1.0 fields (1..9)
    uint32_t startupSafeMs;
    uint32_t allRedMs;
    uint32_t mainMinGreenMs;
    uint32_t mainYellowMs;
    uint32_t sideMinGreenMs;
    uint32_t sideMaxGreenMs;
    uint32_t sideClearGapMs;
    uint32_t sideYellowMs;
    uint32_t pedWalkMs;

    // Extended v2.0.0 fields (10..16)
    uint32_t mainMaxGreenMs;
    uint32_t mainClearGapMs;
    uint32_t turnMinGreenMs;
    uint32_t turnMaxGreenMs;
    uint32_t turnClearGapMs;
    uint32_t turnYellowMs;
    uint32_t pedClearanceMs;
};

constexpr TimingConfig DEFAULT_TIMING_CONFIG = {
    STARTUP_SAFE_MS,
    ALL_RED_MS,
    MAIN_MIN_GREEN_MS,
    MAIN_YELLOW_MS,
    SIDE_MIN_GREEN_MS,
    SIDE_MAX_GREEN_MS,
    SIDE_CLEAR_GAP_MS,
    SIDE_YELLOW_MS,
    PED_WALK_MS,
    MAIN_MAX_GREEN_MS,
    MAIN_CLEAR_GAP_MS,
    TURN_MIN_GREEN_MS,
    TURN_MAX_GREEN_MS,
    TURN_CLEAR_GAP_MS,
    TURN_YELLOW_MS,
    PED_CLEARANCE_MS
};

// Sensor-to-movement mapping & filtering configuration
struct SensorConfig {
    uint8_t  sensorToMovement[IR_SENSOR_COUNT]; // Maps IR1..IR6 to MovementId (0=MAIN, 1=SIDE, 2=TURN)
    uint8_t  sensorValidatedMask;               // Bitmask (bits 0..5) indicating which physical IR sensors are validated
    uint32_t irStabilityMs;                     // Debounce/stability filter duration (ms)
    uint32_t pedDebounceMs;                     // Pushbutton debounce duration (ms)
    uint32_t maxInputStuckMs;                   // Stuck-active timeout (ms)
};

constexpr SensorConfig DEFAULT_SENSOR_CONFIG = {
    {
        static_cast<uint8_t>(MovementId::SIDE_ROAD),      // IR1 (GPIO 33) -> Side Road Primary (preserves v1.1.0)
        static_cast<uint8_t>(MovementId::SIDE_ROAD),      // IR2 (GPIO 34) -> Side Road Secondary
        static_cast<uint8_t>(MovementId::MAIN_ROAD),      // IR3 (GPIO 35) -> Main Road Primary
        static_cast<uint8_t>(MovementId::MAIN_ROAD),      // IR4 (GPIO 36) -> Main Road Secondary
        static_cast<uint8_t>(MovementId::PROTECTED_TURN), // IR5 (GPIO 39) -> Protected Turn Primary
        static_cast<uint8_t>(MovementId::PROTECTED_TURN)  // IR6 (GPIO 15) -> Protected Turn Secondary
    },
    0x00, // No physical IR sensors validated by default until hardware commissioning
    IR_STABILITY_MS,
    PED_DEBOUNCE_MS,
    MAX_INPUT_STUCK_MS
};

// Demand & Fairness Configuration
struct DemandConfig {
    uint32_t waitTimeWeightMs;     // Waiting time (ms) per +1 priority score
    uint8_t  activeSensorWeight;   // Score weight per active sensor on approach
    uint8_t  starvationSkipLimit;  // Max skipped cycles before starvation priority boost
    uint8_t  starvationBoostScore; // Priority score boost when starved
    uint8_t  highDemandSensorThreshold; // Number of active sensors on approach to extend toward maxGreen
};

constexpr DemandConfig DEFAULT_DEMAND_CONFIG = {
    DEFAULT_WAIT_TIME_WEIGHT_MS,
    DEFAULT_ACTIVE_SENSOR_WEIGHT,
    DEFAULT_STARVATION_SKIP_LIMIT,
    DEFAULT_STARVATION_BOOST,
    2
};

// Right-Hand Traffic (Keep-Right) Explicit Movement Conflict Matrix Configuration
// conflicts[i][j] == true means Movement i and Movement j CANNOT be active concurrently.
// By default:
// - All distinct vehicle movements conflict with one another.
// - All vehicle movements conflict with both pedestrian crossings.
// - Protected Turn remains uncommissioned (protectedTurnCommissioned = false) until
//   the physical Keep-Right T-junction lane geometry is confirmed and commissioned.
struct ConflictMatrixConfig {
    bool protectedTurnCommissioned;
    bool rightHandTrafficRule;
    bool conflicts[ static_cast<size_t>(MovementId::COUNT) ][ static_cast<size_t>(MovementId::COUNT) ];
};

constexpr ConflictMatrixConfig DEFAULT_CONFLICT_MATRIX = {
    PROTECTED_TURN_GEOMETRY_VERIFIED, // false by default until geometry confirmed
    RIGHT_HAND_TRAFFIC_RULE,          // true (Keep-Right)
    {
        // MAIN,  SIDE,  TURN,  PED1,  PED2
        { false, true,  true,  true,  true  }, // MAIN_ROAD
        { true,  false, true,  true,  true  }, // SIDE_ROAD
        { true,  true,  false, true,  true  }, // PROTECTED_TURN
        { true,  true,  true,  false, false }, // PED_CROSSING_1 (compatible with PED_CROSSING_2 when all vehicles RED)
        { true,  true,  true,  false, false }  // PED_CROSSING_2
    }
};

} // namespace TrafficConfig

#endif // TRAFFIC_CONFIG_H
