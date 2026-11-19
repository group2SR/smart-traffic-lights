# Full Traffic Control Architecture (`v2.0.0-full-adaptive-control`)

> **Disclaimer:** This controller is an embedded systems prototype and educational engineering platform. It is **not certified for public-road deployment**.

---

## 1. Architectural Overview

The ESP32 Smart Traffic Light Controller uses a strictly non-blocking, `millis()`-driven finite state machine (FSM) with modular separation between traffic-scheduling logic, sensor conditioning, safety invariant verification, persistent storage, and electrical output driving.

```
+-------------------------------------------------------------------------+
|                           ESP32 Main Runtime                            |
|                              (src/main.cpp)                             |
+-------------------+---------------------------------+-------------------+
                    |                                 |
                    v                                 v
      +---------------------------+     +---------------------------+
      |   SerialCommandHandler    |     |     HardwareInputs        |
      | (Non-blocking CLI parser) |     | (6 IR + 2 Button Adapter) |
      +-------------+-------------+     +-------------+-------------+
                    |                                 |
                    +----------------+----------------+
                                     |
                                     v
+-------------------------------------------------------------------------+
|                            TrafficController                            |
|               (include/TrafficController.h, src/TrafficController.cpp)  |
|                                                                         |
|  +----------------------+  +----------------------+  +---------------+  |
|  |   VehicleDetector    |  |    DemandManager     |  |  FaultManager |  |
|  | (6 IR sensors, 100ms |  | (Presence, wait-time |  | (Bitmask, log |  |
|  |  filter, stuck/chat) |  |  & starvation score) |  |  & recovery)  |  |
|  +----------------------+  +----------------------+  +---------------+  |
|                                                                         |
|  +----------------------+  +----------------------+  +---------------+  |
|  | PedestrianController |  | TrafficPhaseManager  |  | Configuration |  |
|  | (2 buttons, 2 RG     |  | (Keep-Right conflict |  |    Storage    |  |
|  |  groups, clearance)  |  |  matrix & modes)     |  | (NVS + CRC)   |  |
|  +----------------------+  +----------------------+  +---------------+  |
+------------------------------------+------------------------------------+
                                     |
                                     | LogicalSignalOutputs (13 channels)
                                     | + verifySafetyInvariants()
                                     v
                    +---------------------------------+
                    |       SignalOutputDriver        |
                    | (Electrical polarity & hardware |
                    |  driver verification gating)    |
                    +---------------------------------+
```

---

## 2. Module Responsibilities

| Module | Header / Source | Responsibility |
| :--- | :--- | :--- |
| **`TrafficConfig`** | `include/TrafficConfig.h` | Centralized GPIO definitions, electrical driver enums, default phase timings, sensor mapping, demand weights, and Right-Hand Traffic (`Keep-Right`) conflict matrix defaults. |
| **`TrafficController`** | `include/TrafficController.h`, `src/TrafficController.cpp` | Top-level non-blocking `millis()` state machine coordinating all phases, operating modes, safety invariant checks, and transitions. |
| **`TrafficPhaseManager`** | `include/TrafficPhaseManager.h` | Enforces the explicit Right-Hand Traffic movement-conflict matrix, gates Protected Turn behind `protectedTurnCommissioned`, and provides the extensible emergency-priority interface. |
| **`VehicleDetector`** | `include/VehicleDetector.h` | Independently conditions all 6 IR vehicle presence sensors (`IR1`–`IR6`) with 100 ms noise filtering, 30 s stuck-sensor isolation, and 1 s oscillation/chatter suppression. |
| **`DemandManager`** | `include/DemandManager.h` | Tracks per-movement vehicle demand (`MAIN_ROAD`, `SIDE_ROAD`, `PROTECTED_TURN`), waiting time (`waitingTimeMs`), skipped-cycle counters (`skippedCycles`), and deterministic adaptive priority scores. |
| **`PedestrianController`** | `include/PedestrianController.h` | Debounces 2 independent pedestrian buttons (`PED1`, `PED2`, 50 ms), latches requests across vehicle phases, manages `WALK`, `CLEARANCE`, and `DONT_WALK` aspects, and prevents simultaneous RED+GREEN. |
| **`SignalOutputDriver`** | `include/SignalOutputDriver.h` | Hardware abstraction layer separating 13 logical lamp states from electrical GPIO levels. Enforces turn-off-before-turn-on sequencing and blocks unverified 5 V common-ground vehicle drivers. |
| **`FaultManager`** | `include/FaultManager.h` | Tracks active fault bitmasks (`FaultCode`), fault timestamps, non-fatal sensor isolation vs. critical latched `FAULT_SAFE` entry, and explicit operator recovery. |
| **`ConfigurationStorage`** | `include/ConfigurationStorage.h` | Persists and validates all controller settings in ESP32 `Preferences` (NVS) using FNV-1a 32-bit checksums and automatic fallback to safe defaults on corruption. |
| **`SerialCommandHandler`** | `include/SerialCommandHandler.h` | Non-blocking command parser supporting mode switching, demand simulation, diagnostics, NVS persistence, and fault recovery. |

---

## 3. Operating Modes

1. **`AUTO` (Default)**:
   - Rests in `MAIN_GREEN` when no conflicting demand exists.
   - Responds to Side Road, commissioned Protected Turn, and Pedestrian requests using deterministic round-robin fairness after minimum green constraints.
2. **`FIXED` (`MODE FIXED`)**:
   - Deterministic fixed-time cycle across enabled movements (`MAIN_GREEN -> SIDE_GREEN -> [TURN_GREEN if commissioned] -> [PED_WALK if requested]`), enforcing configured green, yellow, and all-red durations.
3. **`ACTUATED` (`MODE ACTUATED`)**:
   - Reads all 6 IR sensors independently.
   - Extends active green while vehicles remain detected on that approach up to `maxGreenMs`.
   - Terminates green early after `minGreenMs + clearGapMs` when vehicle presence clears.
   - Never grants green to an approach with zero demand when another approach has demand.
4. **`ADAPTIVE` (`MODE ADAPTIVE`)**:
   - Calculates a deterministic priority score for each waiting movement:
     $$\text{Score}(m) = (\text{activeSensors}(m) \times W_{\text{sensor}}) + \left\lfloor \frac{\text{waitingTimeMs}(m)}{W_{\text{wait}}} \right\rfloor + (\text{skippedCycles}(m) \ge L_{\text{skip}} \ ? \ B_{\text{starve}} : 0)$$
   - Selects the highest-scoring waiting movement at each safe phase boundary and prevents starvation via `skippedCycles` boost and `maxGreenMs` ceilings.
5. **`MANUAL` (`MODE MANUAL`)**:
   - Holds the current phase until the operator issues `NEXT`.
   - Every `NEXT` request is forced through `minGreenMs`, `YELLOW`, and `ALL_RED` clearance intervals—manual control can never bypass safety interlocks.

---

## 4. Implementation & Verification Status Matrix

| Subsystem / Feature | Status | Notes |
| :--- | :--- | :--- |
| Non-blocking `millis()` FSM (`STARTUP_SAFE`, `ALL_RED`, `MAIN`, `SIDE`, `TURN`, `PED_WALK`, `FAULT_SAFE`) | **Implemented and physically verified on target hardware** | Verified by 49 automated Unity tests (`test_main.cpp`) directly on ESP32 on COM5. |
| Operating modes (`AUTO`, `FIXED`, `ACTUATED`, `ADAPTIVE`, `MANUAL`) | **Implemented and tested on hardware** | Production firmware defaults to `MODE FIXED` prototype timings; adaptive mode safety-gated by physical IR sensor commissioning. |
| 6-channel IR sensor filtering, stuck-active timeout, and chatter suppression | **Integrated in firmware; idle checks passed** | 2 independent sensors per road (`Main: 33/34`, `Side: 35/36`, `Turn: 39/15`); idle electrical checks PASS (`RawHigh=1`), awaiting physical obstacle detect-and-release sweeps. |
| Dual pedestrian buttons & dual RG signal groups with configurable clearance | **Commissioned and verified on hardware** | PED1 (19/18, Button 32) and PED2 (16/17, Button 4) operate via ULN2003A low-side sink driver. |
| Explicit Right-Hand Traffic (`RHT / Keep-Right`) conflict matrix & Protected Turn gate | **Configured; Turn held in Safe Red** | `rightHandTrafficRule = true` enforced; `protectedTurnCommissioned = false` keeps Turn in continuous RED until junction geometry confirmed. |
| Vehicle 5V direct ESP32 output driver stage | **Commissioned and physically energizing** | Vehicles wire directly to ESP32 outputs (`DIRECT_ACTIVE_HIGH`, `VEHICLE_DRIVER_VERIFIED = true`). Main Green (GPIO 25) and Side Green (GPIO 14) energize during their phases. |
| Pedestrian LED `ULN2003A` low-side sink driver | **Commissioned and persisted** | Verified on target (`LOW_SIDE_ULN2003A_ACTIVE_HIGH`, `PED_ULN2003A_DRIVER_VERIFIED = true`). |
| Interactive Web Dashboard (`tools/demo_dashboard.html`) | **Implemented and fully synchronized** | Native Web Serial API at 115200 baud; supports live SVG junction, 9 vehicle lamps, 4 pedestrian lamps, 6 IR sensor cards, timings countdown, and controls. |


