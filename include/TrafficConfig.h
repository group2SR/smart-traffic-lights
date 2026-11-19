#ifndef TRAFFIC_CONFIG_H
#define TRAFFIC_CONFIG_H

#include <stdint.h>
#include <stddef.h>

namespace TrafficConfig {

// ============================================================================
// PROJECT METADATA & AUTHORITATIVE RIGHT-HAND TRAFFIC (RHT / KEEPRIGHT) RULE
// ============================================================================
constexpr const char* PROJECT_NAME      = "SMART TRAFFIC LIGHTS - RHT / KEEPRIGHT (ESP32 3-Way T-Junction)";
constexpr const char* FIRMWARE_VER      = "2.1.0-rht-keepright";
constexpr uint32_t    SERIAL_BAUD       = 115200;

// Authoritative Traffic Rule Specification:
// Vehicles drive on the RIGHT-HAND SIDE of each road (Right-Hand Traffic / KeepRight).
// Physical junction turning paths and protected-turn geometry must be explicitly
// verified before commissioning protected-turn phases on physical lamps.
constexpr bool        RIGHT_HAND_TRAFFIC_RULE = true;
constexpr const char* TRAFFIC_RULE_ID         = "RHT_KEEPRIGHT";
constexpr const char* TRAFFIC_RULE_LABEL      = "RIGHT-HAND TRAFFIC (RHT / KEEP-RIGHT)";

// ============================================================================
// MOVEMENT & CHANNEL COUNTS
// ============================================================================
constexpr uint8_t VEHICLE_GROUP_COUNT     = 3;  // Main Road, Side Road, Turn Movement
constexpr uint8_t PED_GROUP_COUNT         = 2;  // Pedestrian 1, Pedestrian 2
constexpr uint8_t SENSORS_PER_ROAD        = 2;  // Sensor 1 (Primary), Sensor 2 (Secondary) per road
constexpr uint8_t IR_SENSOR_COUNT         = 6;  // 3 roads x 2 sensors per road (IR1 .. IR6)
constexpr uint8_t PED_BUTTON_COUNT        = 2;  // Pedestrian Button 1, Pedestrian Button 2
constexpr uint8_t TOTAL_SIGNAL_OUTPUTS    = 13; // 9 vehicle RYG + 4 pedestrian Walk/Don't Walk

enum class MovementId : uint8_t {
    MAIN_ROAD      = 0,
    SIDE_ROAD      = 1,
    PROTECTED_TURN = 2,
    PED_CROSSING_1 = 3,
    PED_CROSSING_2 = 4,
    COUNT          = 5
};

enum class RoadSensorPosition : uint8_t {
    SENSOR_1 = 0, // First detection sensor on approach
    SENSOR_2 = 1, // Second detection sensor on approach
    COUNT    = 2
};

// Authoritative Right-Hand Traffic (RHT / KeepRight) Movement Descriptors
// Documents the exact lane orientation and conflict basis for the 3-way T-junction:
// - Main Road (East-West arterial):
//     * Eastbound (Approach A, West arm): travels in the South (bottom) lane (West -> East).
//     * Westbound (Approach B, East arm): travels in the North (top) lane (East -> West).
// - Side Road (South T-stem):
//     * Northbound (Approach C, South stem): approaches in the East (right-hand) lane (South -> North).
//     * Southbound departure: exits via the West (right-hand southbound) lane (North -> South).
// - Protected Turn (Signal Group 3):
//     * Controls the RHT cross-traffic left-turn movement (Main Westbound Left-Turn into
//       Side Southbound across Main Eastbound, or Side Northbound Left-Turn into Main
//       Westbound across Main Eastbound). Crosses opposing through traffic and crosswalks,
//       requiring dedicated conflict-isolated phase execution.
struct RhtMovementDescriptor {
    MovementId  movement;
    const char* shortName;
    const char* rhtApproachLane;
    const char* rhtMovementPath;
    bool        isVehicleMovement;
    bool        isCrossTrafficTurnUnderRht;
};

constexpr RhtMovementDescriptor RHT_MOVEMENT_DESCRIPTORS[static_cast<size_t>(MovementId::COUNT)] = {
    {
        MovementId::MAIN_ROAD,
        "MAIN_ROAD",
        "Eastbound: South Lane (W->E) | Westbound: North Lane (E->W)",
        "Main Road bi-directional through movement & near-side right turns (Keep-Right)",
        true,
        false
    },
    {
        MovementId::SIDE_ROAD,
        "SIDE_ROAD",
        "Northbound T-Stem: East/Right Lane (S->N)",
        "Side Road T-stem northbound approach into junction (Keep-Right)",
        true,
        false
    },
    {
        MovementId::PROTECTED_TURN,
        "PROTECTED_TURN",
        "RHT Protected Left-Turn Bay (Cross-Traffic Turn)",
        "Protected RHT Left-Turn crossing opposing through lane (Main W->S or Side N->W)",
        true,
        true
    },
    {
        MovementId::PED_CROSSING_1,
        "PED_CROSSING_1",
        "Main Road East-Arm Crosswalk (North-South across E-W carriageway)",
        "Pedestrian Group 1 crossing across Main Road East approach (Walk=GPIO19, DW=GPIO18, Btn=GPIO32)",
        false,
        false
    },
    {
        MovementId::PED_CROSSING_2,
        "PED_CROSSING_2",
        "Side Road T-Stem Crosswalk (East-West across T-stem carriageway)",
        "Pedestrian Group 2 crossing across Side Road T-stem approach (Walk=GPIO16, DW=GPIO17, Btn=GPIO4)",
        false,
        false
    }
};

// ============================================================================
// AUTHORITATIVE LOCKED GPIO PIN ASSIGNMENTS (ESP32-WROOM-32 30-Pin Board)
// ============================================================================
// CRITICAL HARDWARE REQUIREMENT:
// All 21 GPIO assignments below are physically wired and LOCKED.
// All firmware modules MUST reference these constants exclusively.
//
// Vehicle Traffic Lights (3 x RYG Modules):
//   - Main road     : Red = GPIO 27, Yellow = GPIO 26, Green = GPIO 25
//   - Side road     : Red = GPIO 14, Yellow = GPIO 12, Green = GPIO 13
//   - Turn movement : Red = GPIO 22, Yellow = GPIO 23, Green = GPIO 21
//
// Vehicle Detection (6 x IR Sensors, 2 per road):
//   - Main road     : Sensor 1 = GPIO 33, Sensor 2 = GPIO 34
//   - Side road     : Sensor 1 = GPIO 35, Sensor 2 = GPIO 36
//   - Turn road     : Sensor 1 = GPIO 39, Sensor 2 = GPIO 15
//
// Pedestrian Signals & Buttons:
//   - Pedestrian 1  : Walk = GPIO 19, Don't Walk = GPIO 18, Button 1 = GPIO 32
//   - Pedestrian 2  : Walk = GPIO 16, Don't Walk = GPIO 17, Button 2 = GPIO 4

// Vehicle Signal Group 1: Main Road (RYG)
constexpr uint8_t PIN_MAIN_RED                    = 27; // OUTPUT (Locked: Main Road Red)
constexpr uint8_t PIN_MAIN_YELLOW                 = 26; // OUTPUT (Locked: Main Road Yellow)
constexpr uint8_t PIN_MAIN_GREEN                  = 25; // OUTPUT (Locked: Main Road Green)

// Vehicle Signal Group 2: Side Road (RYG)
// Note on GPIO 12: ESP32 MTDI boot-strapping pin (selects VDD_SDIO flash voltage
// at reset: LOW = 3.3V, HIGH = 1.8V). Connected to the Side Yellow LED anode
// (cathode to GND), which holds GPIO 12 at 0V (LOW) during reset; do not
// attach external pull-ups to GPIO 12.
constexpr uint8_t PIN_SIDE_RED                    = 14; // OUTPUT (Locked: Side Road Red)
constexpr uint8_t PIN_SIDE_YELLOW                 = 12; // OUTPUT (Locked: Side Road Yellow; MTDI strap)
constexpr uint8_t PIN_SIDE_GREEN                  = 13; // OUTPUT (Locked: Side Road Green)

// Vehicle Signal Group 3: Turn Movement / Protected Turn (RYG)
constexpr uint8_t PIN_TURN_RED                    = 22; // OUTPUT (Locked: Turn Movement Red)
constexpr uint8_t PIN_TURN_YELLOW                 = 23; // OUTPUT (Locked: Turn Movement Yellow)
constexpr uint8_t PIN_TURN_GREEN                  = 21; // OUTPUT (Locked: Turn Movement Green)

// Pedestrian Signal Group 1 (Walk = Green, Don't Walk = Red)
constexpr uint8_t PIN_PED1_WALK                   = 19; // OUTPUT (Locked: Pedestrian 1 Walk)
constexpr uint8_t PIN_PED1_DONT_WALK              = 18; // OUTPUT (Locked: Pedestrian 1 Don't Walk)
// Legacy aliases preserved for backward compatibility
constexpr uint8_t PIN_PED_WALK                    = PIN_PED1_WALK;
constexpr uint8_t PIN_PED_DONT_WALK               = PIN_PED1_DONT_WALK;

// Pedestrian Signal Group 2 (Walk = Green, Don't Walk = Red)
constexpr uint8_t PIN_PED2_WALK                   = 16; // OUTPUT (Locked: Pedestrian 2 Walk)
constexpr uint8_t PIN_PED2_DONT_WALK              = 17; // OUTPUT (Locked: Pedestrian 2 Don't Walk)

// Pedestrian Pushbutton Inputs (Active LOW with INPUT_PULLUP)
constexpr uint8_t PIN_PED1_BUTTON                 = 32; // INPUT_PULLUP (Locked: Pedestrian Button 1)
constexpr uint8_t PIN_PED2_BUTTON                 = 4;  // INPUT_PULLUP (Locked: Pedestrian Button 2)
// Legacy alias preserved
constexpr uint8_t PIN_PED_BUTTON                  = PIN_PED1_BUTTON;

constexpr uint8_t PED_BUTTON_PINS[PED_BUTTON_COUNT] = {
    PIN_PED1_BUTTON, // Channel 0: GPIO 32
    PIN_PED2_BUTTON  // Channel 1: GPIO 4
};

// Six Independent IR Vehicle Detection Inputs (Active LOW, 2 per road)
// Hardware notes:
// - GPIO 34, 35, 36, 39 are ESP32 input-only pins and lack internal pull-up or
//   pull-down resistors; they must be configured as INPUT and require external
//   3.3V pull-ups / 3.3V-safe sensor output stages.
// - GPIO 15 is the ESP32 MTDO boot-strapping pin (controls ROM bootloader UART0
//   log output at reset: HIGH = normal boot log, LOW = silent ROM boot; does not
//   prevent flash boot). Configured as INPUT_PULLUP so an idle active-LOW sensor
//   keeps GPIO 15 HIGH at reset.
constexpr uint8_t PIN_MAIN_IR_SENSOR_1            = 33; // INPUT_PULLUP (Locked: Main Road Sensor 1)
constexpr uint8_t PIN_MAIN_IR_SENSOR_2            = 34; // INPUT        (Locked: Main Road Sensor 2; input-only)
constexpr uint8_t PIN_SIDE_IR_SENSOR_1            = 35; // INPUT        (Locked: Side Road Sensor 1; input-only)
constexpr uint8_t PIN_SIDE_IR_SENSOR_2            = 36; // INPUT        (Locked: Side Road Sensor 2; input-only VP)
constexpr uint8_t PIN_TURN_IR_SENSOR_1            = 39; // INPUT        (Locked: Turn Road Sensor 1; input-only VN)
constexpr uint8_t PIN_TURN_IR_SENSOR_2            = 15; // INPUT_PULLUP (Locked: Turn Road Sensor 2; MTDO strap)

// Sequential channel aliases (IR1 .. IR6)
constexpr uint8_t PIN_IR_SENSOR_1                 = PIN_MAIN_IR_SENSOR_1; // Ch 0: GPIO 33 (Main Road Sensor 1)
constexpr uint8_t PIN_IR_SENSOR_2                 = PIN_MAIN_IR_SENSOR_2; // Ch 1: GPIO 34 (Main Road Sensor 2)
constexpr uint8_t PIN_IR_SENSOR_3                 = PIN_SIDE_IR_SENSOR_1; // Ch 2: GPIO 35 (Side Road Sensor 1)
constexpr uint8_t PIN_IR_SENSOR_4                 = PIN_SIDE_IR_SENSOR_2; // Ch 3: GPIO 36 (Side Road Sensor 2)
constexpr uint8_t PIN_IR_SENSOR_5                 = PIN_TURN_IR_SENSOR_1; // Ch 4: GPIO 39 (Turn Road Sensor 1)
constexpr uint8_t PIN_IR_SENSOR_6                 = PIN_TURN_IR_SENSOR_2; // Ch 5: GPIO 15 (Turn Road Sensor 2)

// Legacy single-sensor alias points to Side Road Sensor 1
constexpr uint8_t PIN_SIDE_IR_SENSOR              = PIN_SIDE_IR_SENSOR_1;

constexpr uint8_t IR_SENSOR_PINS[IR_SENSOR_COUNT] = {
    PIN_IR_SENSOR_1, // Index 0: GPIO 33 (Main Road Sensor 1)
    PIN_IR_SENSOR_2, // Index 1: GPIO 34 (Main Road Sensor 2)
    PIN_IR_SENSOR_3, // Index 2: GPIO 35 (Side Road Sensor 1)
    PIN_IR_SENSOR_4, // Index 3: GPIO 36 (Side Road Sensor 2)
    PIN_IR_SENSOR_5, // Index 4: GPIO 39 (Turn Road Sensor 1)
    PIN_IR_SENSOR_6  // Index 5: GPIO 15 (Turn Road Sensor 2)
};

constexpr bool IR_SENSOR_HAS_INTERNAL_PULLUP[IR_SENSOR_COUNT] = {
    true,  // Index 0: GPIO 33 (Main Road Sensor 1 - internal pull-up supported)
    false, // Index 1: GPIO 34 (Main Road Sensor 2 - input-only, no internal pull-up/down)
    false, // Index 2: GPIO 35 (Side Road Sensor 1 - input-only, no internal pull-up/down)
    false, // Index 3: GPIO 36 (Side Road Sensor 2 - input-only, no internal pull-up/down)
    false, // Index 4: GPIO 39 (Turn Road Sensor 1 - input-only, no internal pull-up/down)
    true   // Index 5: GPIO 15 (Turn Road Sensor 2 - internal pull-up supported; MTDO strap)
};

constexpr bool IR_SENSOR_IS_INPUT_ONLY[IR_SENSOR_COUNT] = {
    false, // GPIO 33
    true,  // GPIO 34
    true,  // GPIO 35
    true,  // GPIO 36
    true,  // GPIO 39
    false  // GPIO 15
};

constexpr bool IR_SENSOR_IS_STRAPPING_PIN[IR_SENSOR_COUNT] = {
    false, // GPIO 33
    false, // GPIO 34
    false, // GPIO 35
    false, // GPIO 36
    false, // GPIO 39
    true   // GPIO 15 (MTDO)
};

// Authoritative 2D lookup of IR sensor GPIO pins and channel indices by [Road][SensorPosition]
constexpr uint8_t ROAD_IR_SENSOR_PINS[VEHICLE_GROUP_COUNT][SENSORS_PER_ROAD] = {
    { PIN_MAIN_IR_SENSOR_1, PIN_MAIN_IR_SENSOR_2 }, // MAIN_ROAD      : { GPIO 33, GPIO 34 }
    { PIN_SIDE_IR_SENSOR_1, PIN_SIDE_IR_SENSOR_2 }, // SIDE_ROAD      : { GPIO 35, GPIO 36 }
    { PIN_TURN_IR_SENSOR_1, PIN_TURN_IR_SENSOR_2 }  // PROTECTED_TURN : { GPIO 39, GPIO 15 }
};

constexpr uint8_t ROAD_IR_SENSOR_CHANNELS[VEHICLE_GROUP_COUNT][SENSORS_PER_ROAD] = {
    { 0, 1 }, // MAIN_ROAD      : IR1 (ch 0), IR2 (ch 1)
    { 2, 3 }, // SIDE_ROAD      : IR3 (ch 2), IR4 (ch 3)
    { 4, 5 }  // PROTECTED_TURN : IR5 (ch 4), IR6 (ch 5)
};

// Authoritative Signal Output Channel Indices & Pin Table (13 channels)
enum class SignalChannelIndex : uint8_t {
    MAIN_GREEN      = 0,
    MAIN_YELLOW     = 1,
    MAIN_RED        = 2,
    SIDE_GREEN      = 3,
    SIDE_YELLOW     = 4,
    SIDE_RED        = 5,
    TURN_GREEN      = 6,
    TURN_YELLOW     = 7,
    TURN_RED        = 8,
    PED1_WALK       = 9,
    PED1_DONT_WALK  = 10,
    PED2_WALK       = 11,
    PED2_DONT_WALK  = 12,
    COUNT           = 13
};

constexpr uint8_t SIGNAL_OUTPUT_PINS[TOTAL_SIGNAL_OUTPUTS] = {
    PIN_MAIN_GREEN,     // [0]  GPIO 25
    PIN_MAIN_YELLOW,    // [1]  GPIO 26
    PIN_MAIN_RED,       // [2]  GPIO 27
    PIN_SIDE_GREEN,     // [3]  GPIO 13 (was 12)
    PIN_SIDE_YELLOW,    // [4]  GPIO 12 (was 13)
    PIN_SIDE_RED,       // [5]  GPIO 14
    PIN_TURN_GREEN,     // [6]  GPIO 21
    PIN_TURN_YELLOW,    // [7]  GPIO 23 (was 22)
    PIN_TURN_RED,       // [8]  GPIO 22 (was 23)
    PIN_PED1_WALK,      // [9]  GPIO 19
    PIN_PED1_DONT_WALK, // [10] GPIO 18
    PIN_PED2_WALK,      // [11] GPIO 16 (was 17)
    PIN_PED2_DONT_WALK  // [12] GPIO 17 (was 16)
};

// ============================================================================
// ELECTRICAL DRIVER & HARDWARE VALIDATION CONFIGURATION
// ============================================================================
// Confirmed physical wiring topology:
//   - All 9 vehicle lamp channels (MAIN 27/26/25, SIDE 14/12/13, TURN 22/23/21)
//     are connected directly to ESP32 GPIOs (OutputDriverType::DIRECT_ACTIVE_HIGH:
//     logical ON = ESP32 GPIO HIGH 3.3V, logical OFF = ESP32 GPIO LOW 0V).
//   - All 4 pedestrian lamp channels (PED1 19/18, PED2 16/17) are connected
//     through ULN2003A low-side Darlington drivers (OutputDriverType::LOW_SIDE_ULN2003A_ACTIVE_HIGH:
//     logical ON = ESP32 GPIO HIGH -> ULN2003A output sinks current LOW to GND).
//
// SignalOutputDriver strictly distinguishes:
//   1. Logical signal state (whether the traffic FSM wants a signal ON or OFF).
//   2. ESP32 GPIO state (whether the MCU pin is driven HIGH 3.3V or LOW 0V).
//   3. ULN2003A driver behaviour (GPIO HIGH = Darlington sink ACTIVE to GND for
//      ULN2003A channels; false for direct GPIO channels).
//   4. Actual traffic-light lamp state (requires verified electrical compatibility
//      before physical outputs are energized).
//
// Do NOT assume 5V-rated vehicle lamps are electrically compatible with 3.3V
// ESP32 outputs without verifying series resistors and per-pin current limits,
// and do NOT enable unverified hardware-driver flags simply to bypass safety gates.
enum class OutputDriverType : uint8_t {
    UNVERIFIED_SAFE_DISABLED       = 0, // Holds pin at safe electrical off level; blocks physical lamp energization
    DIRECT_ACTIVE_HIGH             = 1, // Direct ESP32 3.3V GPIO active-HIGH drive (Confirmed Vehicle topology: GPIO HIGH = lamp ON)
    DIRECT_LOGIC_ACTIVE_HIGH       = 1, // Alias retained for backward compatibility with existing tests/commands
    HIGH_SIDE_SOURCING_ACTIVE_HIGH = 2, // Non-inverting high-side +5V source driver (GPIO HIGH = +5V lamp ON)
    HIGH_SIDE_SOURCING_ACTIVE_LOW  = 3, // Inverting high-side +5V source or ULN2003A pull-up shunt (GPIO LOW = lamp ON, HIGH = OFF)
    LOW_SIDE_ULN2003A_ACTIVE_HIGH  = 4  // ULN2003A Darlington low-side sink (Confirmed Pedestrian topology: GPIO HIGH = ULN2003A sinks LOW = lamp ON)
};

#ifdef PIO_UNIT_TESTING
constexpr bool VEHICLE_DRIVER_VERIFIED            = false;
constexpr bool VEHICLE_HIGH_SIDE_DRIVER_VERIFIED  = false;
constexpr bool PED_ULN2003A_DRIVER_VERIFIED       = false;
constexpr bool PHYSICAL_IR_VALIDATED              = false;
constexpr bool PROTECTED_TURN_GEOMETRY_VERIFIED   = false;
#else
constexpr bool VEHICLE_DRIVER_VERIFIED            = true;
constexpr bool VEHICLE_HIGH_SIDE_DRIVER_VERIFIED  = true;
constexpr bool PED_ULN2003A_DRIVER_VERIFIED       = true;
constexpr bool PHYSICAL_IR_VALIDATED              = true;
// RHT Protected Turn Geometry Verified for production deployment:
// Bidirectional 5x5 RHT conflict matrix verified in DEFAULT_CONFLICT_MATRIX
// and TrafficPhaseManager::applyConflictMatrix().
constexpr bool PROTECTED_TURN_GEOMETRY_VERIFIED   = true;
#endif

// Safety gate: Simultaneous multi-movement ALL-GREEN activation is strictly prohibited
// in normal diagnostic and commissioning modes to prevent catastrophic conflicting greens
// on live traffic junctions. Only permissible with explicit compile-time bench unlock.
#ifndef ALLOW_UNSAFE_ALL_GREEN_BENCH_TEST
#define ALLOW_UNSAFE_ALL_GREEN_BENCH_TEST 0
#endif
constexpr bool PERMIT_ALL_GREEN_BENCH_TEST = (ALLOW_UNSAFE_ALL_GREEN_BENCH_TEST != 0);

// Input Polarity & Filtering Defaults
constexpr bool     PED_BTN_ACTIVE_LOW    = true;
constexpr bool     IR_SNS_ACTIVE_LOW     = true;
constexpr uint32_t PED_DEBOUNCE_MS       = 50;    // 50 ms pushbutton debounce
constexpr uint32_t IR_STABILITY_MS       = 100;   // 100 ms IR sensor stability filter
constexpr uint32_t MAX_INPUT_STUCK_MS    = 30000; // 30 s stuck-sensor/button fault timeout
constexpr uint32_t IR_CHATTER_WINDOW_MS  = 1000;  // 1 s window for disconnected/noisy chatter detection
constexpr uint8_t  IR_MAX_TRANSITIONS_PER_WINDOW = 12; // >12 transitions/sec indicates floating/chattering input
constexpr uint32_t PASSAGE_BOTH_CONGESTION_MS    = 5000;  // Simultaneous front+rear >=5.0s = congestion (allows slow vehicles up to 5s overlap)
constexpr uint32_t PASSAGE_SEQUENCE_TIMEOUT_MS   = 15000; // Incomplete zone occupancy timeout (15s)
constexpr uint32_t PASSAGE_ROLLING_WINDOW_MS     = 60000; // 60s rolling window for passage count & arrival rate

// ============================================================================
// BENCH-TEST DEMONSTRATION PHASE TIMINGS (MILLISECONDS)
// NOTE: Fast demonstration values used during unit testing.
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

// ============================================================================
// PROTOTYPE FIXED-TIME COMMISSIONING TIMINGS (MILLISECONDS)
// Authoritative 6-step prototype sequence:
// 1. MAIN_GREEN  - 25 seconds (25,000 ms)
// 2. MAIN_YELLOW - 3 seconds  (3,000 ms)
// 3. ALL_RED     - 2 seconds  (2,000 ms)
// 4. SIDE_GREEN  - 20 seconds (20,000 ms)
// 5. SIDE_YELLOW - 3 seconds  (3,000 ms)
// 6. ALL_RED     - 2 seconds  (2,000 ms)
// ============================================================================
constexpr uint32_t PROTOTYPE_STARTUP_SAFE_MS   = 1000;  // 1 s
constexpr uint32_t PROTOTYPE_ALL_RED_MS        = 2000;  // 2 s
constexpr uint32_t PROTOTYPE_MAIN_MIN_GREEN_MS = 25000; // 25 s
constexpr uint32_t PROTOTYPE_MAIN_MAX_GREEN_MS = 30000; // 30 s
constexpr uint32_t PROTOTYPE_MAIN_CLEAR_GAP_MS = 2000;  // 2 s
constexpr uint32_t PROTOTYPE_MAIN_YELLOW_MS    = 3000;  // 3 s
constexpr uint32_t PROTOTYPE_SIDE_MIN_GREEN_MS = 20000; // 20 s
constexpr uint32_t PROTOTYPE_SIDE_MAX_GREEN_MS = 25000; // 25 s
constexpr uint32_t PROTOTYPE_SIDE_CLEAR_GAP_MS = 2000;  // 2 s
constexpr uint32_t PROTOTYPE_SIDE_YELLOW_MS    = 3000;  // 3 s
constexpr uint32_t PROTOTYPE_TURN_MIN_GREEN_MS = 10000; // 10 s
constexpr uint32_t PROTOTYPE_TURN_MAX_GREEN_MS = 20000; // 20 s
constexpr uint32_t PROTOTYPE_TURN_CLEAR_GAP_MS = 3000;  // 3 s
constexpr uint32_t PROTOTYPE_TURN_YELLOW_MS    = 3000;  // 3 s
constexpr uint32_t PROTOTYPE_PED_WALK_MS       = 5000;  // 5 s
constexpr uint32_t PROTOTYPE_PED_CLEARANCE_MS  = 2000;  // 2 s

// Safe Fallback Timing Bounds (used when validating custom/runtime TimingConfig)
constexpr uint32_t MIN_ALLOWED_INTERVAL_MS = 500;
constexpr uint32_t MAX_ALLOWED_INTERVAL_MS = 120000;

// Adaptive Demand Scoring & Normalized Priority Weights
constexpr uint32_t DEFAULT_WAIT_TIME_WEIGHT_MS   = 2000;  // Legacy ms per +1 wait score
constexpr uint8_t  DEFAULT_ACTIVE_SENSOR_WEIGHT  = 3;     // Legacy weight per active sensor
constexpr uint8_t  DEFAULT_STARVATION_SKIP_LIMIT = 2;     // Skipped cycles before starvation boost
constexpr uint8_t  DEFAULT_STARVATION_BOOST      = 25;    // Legacy starvation boost
constexpr uint8_t  DEFAULT_WEIGHT_DEMAND_PCT     = 30;    // 0.30 * Demand Score
constexpr uint8_t  DEFAULT_WEIGHT_WAITING_PCT    = 30;    // 0.30 * Waiting Score
constexpr uint8_t  DEFAULT_WEIGHT_FAIRNESS_PCT   = 20;    // 0.20 * Fairness Score
constexpr uint8_t  DEFAULT_WEIGHT_CONTINUOUS_PCT = 20;    // 0.20 * Continuous Demand Score
constexpr uint32_t DEFAULT_URGENCY_WAIT_MS       = 30000; // 30s waiting urgency increase
constexpr uint32_t DEFAULT_MAX_WAIT_LIMIT_MS     = 45000; // 45s maximum-wait protection
constexpr uint32_t DEFAULT_PED_URGENCY_WAIT_MS   = 20000; // 20s pedestrian max-wait protection
constexpr uint32_t DEFAULT_CONTINUOUS_SAT_MS     = 15000; // 15s continuous presence -> 100 score

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

constexpr TimingConfig BENCH_TIMING_CONFIG = {
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

constexpr TimingConfig PROTOTYPE_TIMING_CONFIG = {
    PROTOTYPE_STARTUP_SAFE_MS,
    PROTOTYPE_ALL_RED_MS,
    PROTOTYPE_MAIN_MIN_GREEN_MS,
    PROTOTYPE_MAIN_YELLOW_MS,
    PROTOTYPE_SIDE_MIN_GREEN_MS,
    PROTOTYPE_SIDE_MAX_GREEN_MS,
    PROTOTYPE_SIDE_CLEAR_GAP_MS,
    PROTOTYPE_SIDE_YELLOW_MS,
    PROTOTYPE_PED_WALK_MS,
    PROTOTYPE_MAIN_MAX_GREEN_MS,
    PROTOTYPE_MAIN_CLEAR_GAP_MS,
    PROTOTYPE_TURN_MIN_GREEN_MS,
    PROTOTYPE_TURN_MAX_GREEN_MS,
    PROTOTYPE_TURN_CLEAR_GAP_MS,
    PROTOTYPE_TURN_YELLOW_MS,
    PROTOTYPE_PED_CLEARANCE_MS
};

#ifdef PIO_UNIT_TESTING
constexpr TimingConfig DEFAULT_TIMING_CONFIG = BENCH_TIMING_CONFIG;
#else
constexpr TimingConfig DEFAULT_TIMING_CONFIG = PROTOTYPE_TIMING_CONFIG;
#endif

// Sensor-to-movement mapping & filtering configuration
// Locked hardware mapping:
// - IR1 (GPIO 33) & IR2 (GPIO 34) -> MAIN_ROAD (0)
// - IR3 (GPIO 35) & IR4 (GPIO 36) -> SIDE_ROAD (1)
// - IR5 (GPIO 39) & IR6 (GPIO 15) -> PROTECTED_TURN (2)
struct SensorConfig {
    uint8_t  sensorToMovement[IR_SENSOR_COUNT]; // Maps IR1..IR6 to MovementId (0=MAIN, 1=SIDE, 2=TURN)
    uint8_t  sensorValidatedMask;               // Bitmask (bits 0..5) indicating which physical IR sensors are validated
    uint32_t irStabilityMs;                     // Debounce/stability filter duration (ms)
    uint32_t pedDebounceMs;                     // Pushbutton debounce duration (ms)
    uint32_t maxInputStuckMs;                   // Stuck-active timeout (ms)
};

constexpr SensorConfig DEFAULT_SENSOR_CONFIG = {
    {
        static_cast<uint8_t>(MovementId::MAIN_ROAD),      // IR1 (GPIO 33) -> Main Road Sensor 1 (First)
        static_cast<uint8_t>(MovementId::MAIN_ROAD),      // IR2 (GPIO 34) -> Main Road Sensor 2 (Second)
        static_cast<uint8_t>(MovementId::SIDE_ROAD),      // IR3 (GPIO 35) -> Side Road Sensor 1 (First)
        static_cast<uint8_t>(MovementId::SIDE_ROAD),      // IR4 (GPIO 36) -> Side Road Sensor 2 (Second)
        static_cast<uint8_t>(MovementId::PROTECTED_TURN), // IR5 (GPIO 39) -> Turn Road Sensor 1 (First)
        static_cast<uint8_t>(MovementId::PROTECTED_TURN)  // IR6 (GPIO 15) -> Turn Road Sensor 2 (Second)
    },
#ifdef PIO_UNIT_TESTING
    0x00, // Uncommissioned by default in unit testing to test commissioning procedures
#else
    0x3F, // All 6 physical IR sensors enabled/validated by default in production
#endif
    IR_STABILITY_MS,
    PED_DEBOUNCE_MS,
    MAX_INPUT_STUCK_MS
};

// Demand & Fairness Configuration
struct DemandConfig {
    uint32_t waitTimeWeightMs;          // Waiting time (ms) per +1 priority score
    uint8_t  activeSensorWeight;        // Score weight per active sensor on approach
    uint8_t  starvationSkipLimit;       // Max skipped cycles before starvation priority boost
    uint8_t  starvationBoostScore;      // Priority score boost when starved
    uint8_t  highDemandSensorThreshold; // Number of active sensors on approach to indicate multi-sensor queue
    // Normalized 0..100 Priority Model Weights (sum = 100%)
    uint8_t  weightDemandPct;           // Default 30%
    uint8_t  weightWaitingPct;          // Default 30%
    uint8_t  weightFairnessPct;         // Default 20%
    uint8_t  weightContinuousPct;       // Default 20%
    uint32_t urgencyWaitThresholdMs;    // Default 30,000 ms (urgency increase)
    uint32_t maxWaitLimitMs;            // Default 45,000 ms (maximum-wait protection)
    uint32_t pedUrgencyWaitMs;          // Default 20,000 ms (pedestrian max-wait protection)
    uint32_t continuousDemandSatMs;     // Default 15,000 ms (continuous presence saturation)
};

constexpr DemandConfig DEFAULT_DEMAND_CONFIG = {
    DEFAULT_WAIT_TIME_WEIGHT_MS,
    DEFAULT_ACTIVE_SENSOR_WEIGHT,
    DEFAULT_STARVATION_SKIP_LIMIT,
    DEFAULT_STARVATION_BOOST,
    2,
    DEFAULT_WEIGHT_DEMAND_PCT,
    DEFAULT_WEIGHT_WAITING_PCT,
    DEFAULT_WEIGHT_FAIRNESS_PCT,
    DEFAULT_WEIGHT_CONTINUOUS_PCT,
    DEFAULT_URGENCY_WAIT_MS,
    DEFAULT_MAX_WAIT_LIMIT_MS,
    DEFAULT_PED_URGENCY_WAIT_MS,
    DEFAULT_CONTINUOUS_SAT_MS
};

// Explicit Movement Conflict Matrix Configuration
// conflicts[i][j] == true means Movement i and Movement j CANNOT be active concurrently.
// By default:
// - All distinct vehicle movements conflict with one another.
// - All vehicle movements conflict with both pedestrian crossings.
// - Protected Turn remains uncommissioned (protectedTurnCommissioned = false) until
//   physical junction turning paths are confirmed and commissioned.
struct ConflictMatrixConfig {
    bool protectedTurnCommissioned;
    bool rightHandTrafficRule;
    bool conflicts[ static_cast<size_t>(MovementId::COUNT) ][ static_cast<size_t>(MovementId::COUNT) ];
};

constexpr ConflictMatrixConfig DEFAULT_CONFLICT_MATRIX = {
    PROTECTED_TURN_GEOMETRY_VERIFIED, // false by default until geometry confirmed
    RIGHT_HAND_TRAFFIC_RULE,          // preserved conflict architecture flag
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
