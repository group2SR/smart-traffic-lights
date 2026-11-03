#ifndef TRAFFIC_CONFIG_H
#define TRAFFIC_CONFIG_H

#include <stdint.h>

namespace TrafficConfig {

// Project Metadata
constexpr const char* PROJECT_NAME    = "ESP32 Smart Traffic Light Controller (3-Way T-Junction)";
constexpr const char* FIRMWARE_VER    = "1.1.0-hardware-ready";
constexpr uint32_t    SERIAL_BAUD     = 115200;

// ============================================================================
// CENTRALIZED GPIO PIN ASSIGNMENTS
// ============================================================================
// Signal Outputs (Active HIGH when physical LEDs are wired; initialized LOW)
// Note: Main road serves the two opposing through-approaches (Approach A & B)
// synchronously; Side road serves the terminating T-stem approach (Approach C).
constexpr uint8_t PIN_MAIN_GREEN      = 27; // OUTPUT (Safe general-purpose GPIO)
constexpr uint8_t PIN_MAIN_YELLOW     = 26; // OUTPUT (Safe general-purpose GPIO)
constexpr uint8_t PIN_MAIN_RED        = 25; // OUTPUT (Safe general-purpose GPIO)

constexpr uint8_t PIN_SIDE_GREEN      = 13; // OUTPUT (Safe general-purpose GPIO)
constexpr uint8_t PIN_SIDE_YELLOW     = 12; // OUTPUT (Note: MTDI boot-strapping pin; see hardware notes)
constexpr uint8_t PIN_SIDE_RED        = 14; // OUTPUT (Safe general-purpose GPIO)

constexpr uint8_t PIN_PED_WALK        = 19; // OUTPUT (Safe general-purpose GPIO)
constexpr uint8_t PIN_PED_DONT_WALK   = 18; // OUTPUT (Safe general-purpose GPIO)

// Physical Inputs (Configured with INPUT_PULLUP)
// External wiring requirements:
// - GPIO 33 (Side-road IR sensor): Connect 3.3V open-collector/digital active-LOW output
//   with a 10k-ohm external pull-up to 3.3V recommended for long cable runs.
// - GPIO 32 (Pedestrian button): Momentary normally-open switch to GND (internal pull-up enabled;
//   external 10k-ohm pull-up to 3.3V and 100nF ceramic debounce capacitor recommended).
constexpr uint8_t PIN_SIDE_IR_SENSOR  = 33; // INPUT_PULLUP (Active LOW when vehicle detected)
constexpr uint8_t PIN_PED_BUTTON      = 32; // INPUT_PULLUP (Active LOW when pressed)

// Input Polarity & Filtering
// Intended physical polarity: LOW when pressed / vehicle detected
constexpr bool    PED_BTN_ACTIVE_LOW  = true;
constexpr bool    IR_SNS_ACTIVE_LOW   = true;

// Whether the physical IR sensor module has been validated in hardware.
// Keep false until the physical IR sensor on GPIO 33 is connected and verified,
// preventing unvalidated/disconnected/floating pins from generating false detections.
constexpr bool    PHYSICAL_IR_VALIDATED = false;

constexpr uint32_t PED_DEBOUNCE_MS      = 50;    // ~50 ms stable-state button debounce
constexpr uint32_t IR_STABILITY_MS      = 100;   // 100 ms stability filter for IR sensor
constexpr uint32_t MAX_INPUT_STUCK_MS   = 30000; // 30 s stuck-sensor/button fault timeout

// ============================================================================
// BENCH-TEST DEMONSTRATION PHASE TIMINGS (MILLISECONDS)
// NOTE: Demonstration values only — NOT certified for public-road use.
// ============================================================================
constexpr uint32_t STARTUP_SAFE_MS    = 1000; // Startup safe interval before ALL_RED
constexpr uint32_t ALL_RED_MS         = 1000; // All-red clearance interval (1 s)
constexpr uint32_t MAIN_MIN_GREEN_MS  = 4000; // Minimum main-road green (4 s)
constexpr uint32_t MAIN_YELLOW_MS     = 2000; // Main-road yellow (2 s)
constexpr uint32_t SIDE_MIN_GREEN_MS  = 2000; // Minimum side-road green (2 s)
constexpr uint32_t SIDE_MAX_GREEN_MS  = 8000; // Maximum side-road green (8 s)
constexpr uint32_t SIDE_CLEAR_GAP_MS  = 2000; // Side-road vehicle clear-gap (2 s)
constexpr uint32_t SIDE_YELLOW_MS     = 2000; // Side-road yellow (2 s)
constexpr uint32_t PED_WALK_MS        = 5000; // Pedestrian WALK interval (5 s)

// Safe Fallback Timing Bounds (used when validating custom/runtime TimingConfig)
constexpr uint32_t MIN_ALLOWED_INTERVAL_MS = 500;
constexpr uint32_t MAX_ALLOWED_INTERVAL_MS = 120000;

struct TimingConfig {
    uint32_t startupSafeMs;
    uint32_t allRedMs;
    uint32_t mainMinGreenMs;
    uint32_t mainYellowMs;
    uint32_t sideMinGreenMs;
    uint32_t sideMaxGreenMs;
    uint32_t sideClearGapMs;
    uint32_t sideYellowMs;
    uint32_t pedWalkMs;
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
    PED_WALK_MS
};

} // namespace TrafficConfig

#endif // TRAFFIC_CONFIG_H
