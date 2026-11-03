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
| `sensorToMovement[0..5]` | `{1, 1, 0, 0, 2, 2}` | `0 .. 2` | Maps `IR1..IR6` to `MAIN_ROAD (0)`, `SIDE_ROAD (1)`, `PROTECTED_TURN (2)`. |
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
| `CONFIG` | Display hardware driver types, verification status, and controller parameters. |
| `SAVE` | Validate and save current configuration to ESP32 NVS. |
| `LOAD` | Load and validate configuration from ESP32 NVS. |
| `DEFAULTS` | Reset configuration to documented defaults and save to NVS. |
| `REBOOT` | Perform safe logical reset and restart the ESP32. |

