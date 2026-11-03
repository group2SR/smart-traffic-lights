# Traffic Phases and Right-Hand Traffic (Keep-Right) Conflict Matrix

---

## 1. Traffic Rule & Junction Geometry Policy

- **Traffic Rule**: **RIGHT-HAND TRAFFIC / KEEP-RIGHT** (`TrafficConfig::RIGHT_HAND_TRAFFIC_RULE = true`).
- **No Unverified Geometry Assumptions**:
  - At a 3-way T-junction under Right-Hand Traffic, vehicles drive on the right-hand side of each carriageway.
  - Because the physical T-junction lane layout and turning paths have not yet been physically commissioned, **Protected Turn (`MovementId::PROTECTED_TURN`) is disabled by default (`protectedTurnCommissioned = false`)**.
  - The controller never infers that two movements are compatible merely because they belong to different RYG signal groups.

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

