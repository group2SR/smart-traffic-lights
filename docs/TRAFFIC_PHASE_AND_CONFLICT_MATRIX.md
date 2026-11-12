# Traffic Phases and Right-Hand Traffic (Keep-Right) Conflict Matrix

---

## 1. Traffic Rule & Right-Hand Traffic (KeepRight) Junction Geometry

- **Authoritative Traffic Rule**: **RIGHT-HAND TRAFFIC / KEEP-RIGHT (`RHT_KEEPRIGHT`)** (`TrafficConfig::RIGHT_HAND_TRAFFIC_RULE = true`).
- **Physical Rule**: **Vehicles drive on the RIGHT-HAND SIDE of each road.**
- **Documented 3-Way T-Junction Geometry (`RHT_MOVEMENT_DESCRIPTORS`)**:
  1. **`MAIN_ROAD` (Movement 0 — East-West Arterial, RYG `GPIO 27, 26, 25`, IR `GPIO 33, 34`)**:
     - **Eastbound (West Approach A)**: Vehicles travel in the **South (Bottom) Lane** (`West -> East`), keeping to the right of the Main Road centerline.
     - **Westbound (East Approach B)**: Vehicles travel in the **North (Top) Lane** (`East -> West`), keeping to the right of the Main Road centerline.
  2. **`SIDE_ROAD` (Movement 1 — South T-Stem Approach, RYG `GPIO 13, 12, 14`, IR `GPIO 35, 36`)**:
     - **Northbound (Approach C)**: Vehicles approach the junction in the **East (Right-Hand) Lane** of the South T-stem (`South -> North`).
     - **Southbound Departure**: Vehicles turning into the Side Road depart via the **West (Right-Hand Southbound) Lane** of the T-stem (`North -> South`).
  3. **`PROTECTED_TURN` (Movement 2 — RHT Cross-Traffic Left-Turn Phase, RYG `GPIO 23, 22, 21`, IR `GPIO 39, 15`)**:
     - Under Right-Hand Traffic (KeepRight), a **left turn** is the cross-traffic turn that crosses the opposing lane (specifically, **Main Road Westbound turning left/South into Side Road**, or **Side Road Northbound turning left/West into Main Road Westbound**).
     - Because an RHT left turn crosses opposing through lanes and pedestrian crosswalk paths, it conflicts with `MAIN_ROAD`, `SIDE_ROAD`, `PED_CROSSING_1`, and `PED_CROSSING_2`.
     - **Identified Physical Mounting Ambiguity & Safety Gate**: While the electrical pins (`RYG = GPIO 23, 22, 21`; `IR = GPIO 39, 15`) and conflict isolation rules are fixed, whether the 3rd RYG head is physically mounted on the Main Westbound Left-Turn bay or the Side Northbound Left-Turn bay must be verified on the physical junction model before enabling `PROTECTED_TURN` on live lamps (`PROTECTED_TURN_GEOMETRY_VERIFIED = false` / `protectedTurnCommissioned = false` by default).
  4. **`PED_CROSSING_1` (Movement 3 — Main Road East-Arm Crosswalk, `Walk=GPIO 19, DW=GPIO 18, Button=GPIO 32`)**:
     - Crosses the Main Road East carriageway North-South; conflicts with all vehicle movements (`MAIN_ROAD`, `SIDE_ROAD`, `PROTECTED_TURN`).
  5. **`PED_CROSSING_2` (Movement 4 — Side Road T-Stem Crosswalk, `Walk=GPIO 17, DW=GPIO 16, Button=GPIO 4`)**:
     - Crosses the Side Road T-stem approach East-West; conflicts with all vehicle movements (`MAIN_ROAD`, `SIDE_ROAD`, `PROTECTED_TURN`) and is compatible with `PED_CROSSING_1` during `PED_WALK` when all three vehicle groups display `RED` (`GPIO 27, 13, 23`).

---

## 2. Supported Traffic States & Phase Transitions

| State | Active Movement | Vehicle Group 1 (`MAIN`) | Vehicle Group 2 (`SIDE`) | Vehicle Group 3 (`TURN`) | Pedestrian Group 1 (`PED1`) | Pedestrian Group 2 (`PED2`) |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: |
| **`STARTUP_SAFE`** | None (Safe Boot) | OFF | OFF | OFF | OFF | OFF |
| **`ALL_RED`** | Clearance / Ped Clearance | `RED` | `RED` | `RED` | `DONT_WALK (RED)` | `DONT_WALK (RED)` |
| **`MAIN_GREEN`** | Main Road | `GREEN` | `RED` | `RED` | `DONT_WALK (RED)` | `DONT_WALK (RED)` |
| **`MAIN_YELLOW`** | Main Clearance | `YELLOW` | `RED` | `RED` | `DONT_WALK (RED)` | `DONT_WALK (RED)` |
| **`SIDE_GREEN`** | Side Road | `RED` | `GREEN` | `RED` | `DONT_WALK (RED)` | `DONT_WALK (RED)` |
| **`SIDE_YELLOW`** | Side Clearance | `RED` | `YELLOW` | `RED` | `DONT_WALK (RED)` | `DONT_WALK (RED)` |
| **`TURN_GREEN`*** | Protected Turn | `RED` | `RED` | `GREEN` | `DONT_WALK (RED)` | `DONT_WALK (RED)` |
| **`TURN_YELLOW`*** | Turn Clearance | `RED` | `RED` | `YELLOW` | `DONT_WALK (RED)` | `DONT_WALK (RED)` |
| **`PED_WALK`** | Pedestrian Crossings | `RED` | `RED` | `RED` | `WALK` (if req) | `WALK` (if req) |
| **`FAULT_SAFE`** | Latched Fault State | OFF | OFF | OFF | OFF | OFF |

*\*Note: `TURN_GREEN` and `TURN_YELLOW` are only reachable when `protectedTurnCommissioned == true`.*

### Mandatory Transition Rules
1. Every green phase (`MAIN_GREEN`, `SIDE_GREEN`, `TURN_GREEN`) must complete its minimum green duration (`minGreenMs`) before transitioning.
2. Every green phase must transition to its corresponding yellow phase (`MAIN_YELLOW`, `SIDE_YELLOW`, `TURN_YELLOW`) — never directly to `ALL_RED` or another green.
3. Every yellow phase must transition to `ALL_RED` for at least `allRedMs` before any conflicting movement receives green or walk.
4. `PED_WALK` must transition to `ALL_RED` for at least `pedClearanceMs` (`isPedestrianClearanceActive() == true`, both pedestrian signals showing `DONT_WALK` while all vehicle groups remain `RED`) before any vehicle phase receives green.

---

## 3. Explicit Movement Conflict Matrix (`ConflictMatrixConfig`)

In `TrafficConfig::DEFAULT_CONFLICT_MATRIX`, `true` (`CONFLICT`) indicates that the two movements are mutually exclusive and **must never be active simultaneously**:

| Movement | `MAIN_ROAD` (0) | `SIDE_ROAD` (1) | `PROTECTED_TURN` (2) | `PED_CROSSING_1` (3) | `PED_CROSSING_2` (4) |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **`MAIN_ROAD` (0)** | `false` | **`true` (CONFLICT)** | **`true` (CONFLICT)** | **`true` (CONFLICT)** | **`true` (CONFLICT)** |
| **`SIDE_ROAD` (1)** | **`true` (CONFLICT)** | `false` | **`true` (CONFLICT)** | **`true` (CONFLICT)** | **`true` (CONFLICT)** |
| **`PROTECTED_TURN` (2)** | **`true` (CONFLICT)** | **`true` (CONFLICT)** | `false` | **`true` (CONFLICT)** | **`true` (CONFLICT)** |
| **`PED_CROSSING_1` (3)** | **`true` (CONFLICT)** | **`true` (CONFLICT)** | **`true` (CONFLICT)** | `false` | `false` (Compatible during All-Vehicle-Red) |
| **`PED_CROSSING_2` (4)** | **`true` (CONFLICT)** | **`true` (CONFLICT)** | **`true` (CONFLICT)** | `false` (Compatible during All-Vehicle-Red) | `false` |

### Validation Rules (`TrafficPhaseManager::applyConflictMatrix`)
- Any configuration with `rightHandTrafficRule == false` is rejected.
- Any asymmetric matrix (`conflicts[i][j] != conflicts[j][i]`) is rejected.
- Any matrix that disables core vehicle-to-vehicle or vehicle-to-pedestrian conflicts is rejected and sanitized back to `DEFAULT_CONFLICT_MATRIX`.

---

## 4. Verification Status

- **Implemented and tested in software**: Phase transition rules, conflict matrix validation, and uncommissioned Protected Turn blocking (`test_every_permitted_phase_transition_including_turn`, `test_protected_turn_conflict_prevention_and_commissioning_gate`).
- **Configured but not physically validated**: Physical Right-Hand Traffic lane assignments for `PROTECTED_TURN` (`protectedTurnCommissioned = false` by default).

