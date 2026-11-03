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
| Non-blocking `millis()` FSM (`STARTUP_SAFE`, `ALL_RED`, `MAIN`, `SIDE`, `TURN`, `PED_WALK`, `FAULT_SAFE`) | **Implemented and tested in software** | Verified by 31 automated Unity tests (`test_main.cpp`). |
| Operating modes (`AUTO`, `FIXED`, `ACTUATED`, `ADAPTIVE`, `MANUAL`) | **Implemented and tested in software** | Verified in unit tests 16, 17, 18, 19, 26, and 30. |
| 6-channel IR sensor filtering, stuck-active timeout, and chatter suppression | **Implemented and tested in software** | Software logic tested; physical sensors remain unvalidated (`sensorValidatedMask = 0x00`). |
| Dual pedestrian buttons & dual RG signal groups with configurable clearance | **Implemented and tested in software** | Verified in unit tests 4, 10, 23, and 24. |
| Explicit Right-Hand Traffic (`Keep-Right`) conflict matrix & Protected Turn gate | **Configured but not physically validated** | `protectedTurnCommissioned = false` by default until physical junction lane geometry is confirmed. |
| Vehicle 5 V common-ground high-side sourcing driver stage | **Blocked pending hardware verification** | `ULN2003A` is a low-side sink and cannot source `+5V` to common-ground vehicle modules. Physical vehicle outputs remain disabled (`VEHICLE_HIGH_SIDE_DRIVER_VERIFIED = false`) until a high-side driver is wired. |
| Pedestrian LED `ULN2003A` low-side driver (common-anode + series resistors) | **Configured but not physically validated** | Compatible with `ULN2003A` when wired common-anode to `+5V` with individual current-limiting resistors. |
| Hardware emergency vehicle transponder/siren receiver | **Not yet implemented** | Extensible software interface (`requestEmergencyPriority`) is implemented and rejects ordinary IR sensors; physical receiver hardware is not present. |

