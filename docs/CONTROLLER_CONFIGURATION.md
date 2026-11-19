# Controller Configuration & NVS Persistence Guide

---

## 1. Persistent Configuration Overview (`ConfigurationStorage`)

The controller stores its runtime configuration in ESP32 non-volatile storage (`Preferences` namespace `"stl_cfg_v2"`, key `"ctrl_blob"`) encapsulated in `PersistentControllerConfig` (`include/ConfigurationStorage.h`).

Every saved configuration includes:
- Magic header: `0x53544C32` (`"STL2"`)
- Schema version: `2`
- 32-bit FNV-1a integrity checksum (`checksum`)

If NVS data is uninitialized, corrupted, or contains any out-of-range timing, sensor, demand, driver, or conflict-matrix value, `ConfigurationStorage::load()` automatically rejects the blob, restores `ConfigurationStorage::makeDefaultConfig()`, and logs `CONFIG_FALLBACK`.

---

## 2. Configurable Parameters & Safe Defaults

### 2.1 Phase Timing Parameters (`TimingConfig`)

| Parameter | Default (`ms`) | Allowed Range (`ms`) | Constraint / Description |
| :--- | :---: | :---: | :--- |
| `startupSafeMs` | `1000` | `500 .. 120000` | Initial de-energized startup interval before `ALL_RED`. |
| `allRedMs` | `1000` | `500 .. 120000` | All-red clearance interval between conflicting vehicle phases. |
| `mainMinGreenMs` | `4000` | `500 .. 120000` | Minimum green time for Main Road (`<= mainMaxGreenMs`). |
| `mainMaxGreenMs` | `12000` | `500 .. 120000` | Maximum green time for Main Road when conflicting demand waits. |
| `mainClearGapMs` | `2000` | `500 .. 120000` | Vehicle clear-gap extension interval for Main Road. |
| `mainYellowMs` | `2000` | `500 .. 120000` | Yellow clearance interval for Main Road. |
| `sideMinGreenMs` | `2000` | `500 .. 120000` | Minimum green time for Side Road (`<= sideMaxGreenMs`). |
| `sideMaxGreenMs` | `8000` | `500 .. 120000` | Maximum green time for Side Road. |
| `sideClearGapMs` | `2000` | `500 .. 120000` | Vehicle clear-gap extension interval for Side Road. |
| `sideYellowMs` | `2000` | `500 .. 120000` | Yellow clearance interval for Side Road. |
| `turnMinGreenMs` | `2000` | `500 .. 120000` | Minimum green time for Protected Turn (`<= turnMaxGreenMs`). |
| `turnMaxGreenMs` | `6000` | `500 .. 120000` | Maximum green time for Protected Turn. |
| `turnClearGapMs` | `2000` | `500 .. 120000` | Vehicle clear-gap extension interval for Protected Turn. |
| `turnYellowMs` | `2000` | `500 .. 120000` | Yellow clearance interval for Protected Turn. |
| `pedWalkMs` | `5000` | `500 .. 120000` | Minimum pedestrian `WALK` duration. |
| `pedClearanceMs` | `1000` | `500 .. 120000` | Post-`WALK` pedestrian clearance duration (`DONT_WALK` + all vehicles `RED`). |

### 2.2 Sensor & Input Filtering (`SensorConfig`)

| Parameter | Default | Allowed Range | Description |
| :--- | :---: | :---: | :--- |
| `sensorToMovement[0..5]` | `{0, 0, 1, 1, 2, 2}` | `0 .. 2` | Maps `IR1..IR6` (`GPIO 33, 34, 35, 36, 39, 15`) to `MAIN_ROAD (0)`, `SIDE_ROAD (1)`, `PROTECTED_TURN (2)` (2 independent sensors per road). |
| `sensorValidatedMask` | `0x00` | `0x00 .. 0x3F` | Bitmask enabling validated physical IR sensors (`0x00` = unvalidated safe default). |
| `irStabilityMs` | `100 ms` | `10 .. 5000 ms` | Stability filter window before an IR state change is accepted. |
| `pedDebounceMs` | `50 ms` | `10 .. 2000 ms` | Pushbutton debounce window for `PED1` and `PED2`. |
| `maxInputStuckMs` | `30000 ms` | `1000 .. 600000 ms` | Continuous active timeout before flagging a sensor/button stuck fault. |

### 2.3 Adaptive Demand Scoring (`DemandConfig`)

| Parameter | Default | Allowed Range | Description |
| :--- | :---: | :---: | :--- |
| `waitTimeWeightMs` | `2000 ms` | `100 .. 60000 ms` | Waiting time required to add `+1` to an approach's priority score. |
| `activeSensorWeight` | `3` | `1 .. 50` | Priority score added per active sensor on an approach. |
| `starvationSkipLimit` | `2` | `1 .. 20` | Number of skipped cycles before applying `starvationBoostScore`. |
| `starvationBoostScore` | `25` | `0 .. 255` | Priority score boost applied to prevent starvation. |

---

## 3. Serial Command Reference (`115200` Baud)

| Command | Action |
| :--- | :--- |
| `HELP` | Print available serial commands. |
| `STATUS` | Print current state, operating mode, control mode, demands, 13 logical outputs, and fault status. |
| `MODE AUTO` | Switch to default automatic mode. |
| `MODE FIXED` | Switch to fixed-time cyclic mode. |
| `MODE ACTUATED` | Switch to 6-sensor vehicle-actuated mode. |
| `MODE ADAPTIVE` | Switch to adaptive demand & waiting-time priority mode. |
| `MODE MANUAL` | Switch to manual operator step mode. |
| `NEXT` | Advance to the next phase in `MODE MANUAL` (via safe yellow and all-red clearance). |
| `PED 1` / `PED 2` | Queue simulated pedestrian crossing request on Group 1 or Group 2. |
| `CAR ON` / `CAR OFF` | Set or clear simulated Side Road vehicle demand. |
| `CAR MAIN ON` / `OFF` | Set or clear simulated Main Road vehicle demand. |
| `CAR TURN ON` / `OFF` | Set or clear simulated Protected Turn vehicle demand. |
| `FAULT` | Immediately latch `FAULT_SAFE` state (`OPERATOR_INJECTED_FAULT`). |
| `CLEAR` | Recover from `FAULT_SAFE` via `STARTUP_SAFE -> ALL_RED`, or clear simulated demands. |
| `SENSORS` | Display status, validation, and fault flags for `IR1..IR6` and `PED1..PED2`. |
| `DEMAND` | Display approach demand, waiting time, skipped cycles, and adaptive scores. |
| `PHASES` | Display movement phases and the active Right-Hand Traffic conflict matrix. |
| `TIMING` | Display all configured timing intervals. |
| `TIMING PROTOTYPE` | Load prototype timing defaults (Main 25s, Side 20s, Yellow 3s, All-Red 2s). |
| `TIMING BENCH` | Load bench demo timings (Main 4s, Side 2s, Yellow 2s, All-Red 1s). |
| `TIMING MAIN <minG> [yel]` | Configure Main Road minimum green and optional yellow clearance duration (ms). |
| `TIMING SIDE <minG> [yel]` | Configure Side Road minimum green and optional yellow clearance duration (ms). |
| `TIMING ALL_RED <ms>` | Configure all-red intersection clearance interval (ms). |
| `TIMING PED <walk> [clear]` | Configure pedestrian walk and clearance durations (ms). |
| `COMMISSION IR <1-6> ON/OFF` | Enable or disable physical commissioning on a specific IR channel. |
| `COMMISSION IR ALL ON/OFF` | Enable or disable physical commissioning on all 6 IR channels. |
| `CONFIG` | Display hardware driver types, verification status, and controller parameters. |
| `SAVE` | Validate and save current configuration to ESP32 NVS. |
| `LOAD` | Load and validate configuration from ESP32 NVS. |
| `DEFAULTS` | Reset configuration to documented defaults and save to NVS. |
| `REBOOT` | Perform safe logical reset and restart the ESP32. |

---

## 4. Web Dashboard (`tools/demo_dashboard.html`)

The controller includes an interactive, browser-based monitoring and commissioning dashboard:

- **Location:** `tools/demo_dashboard.html`
- **Transport:** Native Web Serial API at `115,200 baud` (Google Chrome / Microsoft Edge on Desktop).
- **Zero Dependencies:** Pure HTML5/CSS3/SVG/ES6 single-file architecture with zero external CDN dependencies.
- **Features Supported:**
  1. **T-Junction Live Visualization:** Real-time top-down SVG rendering with Right-Hand Traffic lane geometry, active signal lamps, and 6 IR sensor indicators.
  2. **Vehicle Signal Status Panel:** Full status indicators for Main Road (27/26/25), Side Road (13/12/14), and Protected Turn (23/22/21) with clear `DISABLED / NOT COMMISSIONED` status badge.
  3. **Dual Pedestrian Crossings:** Split monitoring for Pedestrian Crossing 1 (Walk 19, DW 18, Button 32) and Crossing 2 (Walk 16, DW 17, Button 4).
  4. **Six IR Vehicle Detection Sensors:** Dedicated 6-channel status grid displaying GPIOs (33, 34, 35, 36, 39, 15), live raw pin readings, stable detection states, detection/release counts, cycle observed flags, stuck/chatter fault indicators, and per-channel commissioning toggles.
  5. **Operating Mode Management:** Selection between `MODE FIXED`, `MODE ADAPTIVE` (safety-gated until all 6 IR channels are commissioned), and `MODE MANUAL` (with phase step advancement).
  6. **Live Remaining Green Countdown:** Real-time progress bar and countdown timer synchronized with firmware elapsed time and configured phase limits.
  7. **Controller Health & Diagnostics:** Serial health monitoring, stale telemetry detection (>2.8s), active fault mask decoding, recoverable fault clearing, logical reset confirmation, and NVS persistence controls.


