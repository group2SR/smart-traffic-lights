# Hardware Commissioning and Electrical Safety Checklist

> **CRITICAL RULE:** Do not energize physical lamp modules or enable unverified sensor channels until the corresponding hardware checks below are completed and verified with a multimeter.

---

## 1. Current Commissioning Status Summary

| Subsystem | Confirmed Topology | Commissioning Gate in Firmware |
| :--- | :--- | :--- |
| **Core FSM, RHT Rules, Modes & Safety Invariants** | **Implemented & Tested on ESP32 (`COM5`)** | Active (`49 / 49` Unity unit tests passing on ESP32 target). |
| **3 × Vehicle RYG Traffic Light Modules (9 Channels)** | **Direct ESP32 GPIO Outputs (`DIRECT_ACTIVE_HIGH`)** | **COMMISSIONED & PERSISTED IN NVS** (Electrical audit, all 9 individual lamps, clearance sequence, and reboot verified on `COM5`). |
| **2 × Pedestrian RED/GREEN Signal Groups (4 Channels)** | **`ULN2003A` Low-Side Driver (`LOW_SIDE_ULN2003A_ACTIVE_HIGH`)** | **COMMISSIONED & PERSISTED IN NVS** (Physical lamp illumination & mutual exclusion verified by operator). |
| **6 × IR Vehicle Presence Sensors (`IR1..IR6`)** | **Idle Electrical Check PASS (`RawHigh=1`), Safety Gate Active** | `sensorValidatedMask = 0x00` (Safety Gate enforces `CycleObserved = YES` per channel before commissioning). |
| **Protected Turn Movement (`TURN_GREEN`)** | **Configured, Pending Physical Turn-Bay Geometry Check** | `protectedTurnCommissioned = false` (Right-Hand Traffic / Keep-Right). |

---

## 2. Electrical Audit of Confirmed Output Topologies

### 2.1 Vehicle Signal Output Audit (9 Channels: `MAIN`, `SIDE`, `TURN` — Direct ESP32 Outputs)
- **Confirmed Physical Wiring**:
  - The 9 vehicle lamp channels do **not** pass through the `ULN2003A`. They are connected directly to the ESP32 GPIOs:
    - **Traffic Light 1 (`MAIN`)**: Red = `GPIO 27`, Yellow = `GPIO 26`, Green = `GPIO 25`
    - **Traffic Light 2 (`SIDE`)**: Red = `GPIO 14`, Yellow = `GPIO 12`, Green = `GPIO 13`
    - **Traffic Light 3 (`TURN`)**: Red = `GPIO 22`, Yellow = `GPIO 23`, Green = `GPIO 21`
- **Polarity (`OutputDriverType::DIRECT_ACTIVE_HIGH`)**:
  - Logical `ON` $\rightarrow$ ESP32 GPIO driven `HIGH` (`3.3V`), `ulnSinkActive = false`, lamp energized (`true` when commissioned).
  - Logical `OFF` $\rightarrow$ ESP32 GPIO driven `LOW` (`0V`), `ulnSinkActive = false`, lamp dark (`false`).
- **Pre-Commissioning Electrical Check**:
  - Verify series current-limiting resistors ($\ge 68\text{–}100\text{ }\Omega$) are present on all 9 channels so per-pin ESP32 GPIO source current stays within $\le 12\text{–}20\text{ mA}$ at $3.3\text{ V}$.
  - Verified: Common connection is Common-Cathode (Common Ground to `GND`). No GPIO is exposed to 5V.
- **Physical Commissioning Status**:
  - **CONFIRMED & COMMISSIONED**: All 9 vehicle lamps tested individually and verified on `ESP32-WROOM-32 30-Pin Board` (`COM5`).
  - Saved to ESP32 persistent NVS (`COMMISSION VEH ON` + `SAVE`) and verified across multiple reboots.
  - Active configuration reports `Vehicle Driver: DIRECT_ACTIVE_HIGH (Verified=YES)`.
  - Individual lamp test results:
    - `MAIN_R` (GPIO 27): `PASS` (Illuminated individually, all other 12 lamps OFF)
    - `MAIN_Y` (GPIO 26): `PASS` (Illuminated individually, all other 12 lamps OFF)
    - `MAIN_G` (GPIO 25): `PASS` (Illuminated individually, all other 12 lamps OFF)
    - `SIDE_R` (GPIO 14): `PASS` (Illuminated individually, all other 12 lamps OFF)
    - `SIDE_Y` (GPIO 12): `PASS` (Illuminated individually, all other 12 lamps OFF; MTDI strap intact)
    - `SIDE_G` (GPIO 13): `PASS` (Illuminated individually, all other 12 lamps OFF)
    - `TURN_R` (GPIO 22): `PASS` (Illuminated individually, all other 12 lamps OFF)
    - `TURN_Y` (GPIO 23): `PASS` (Illuminated individually, all other 12 lamps OFF)
    - `TURN_G` (GPIO 21): `PASS` (Illuminated individually, all other 12 lamps OFF)
  - Traffic sequence verified:
    `MAIN_GREEN -> MAIN_YELLOW (2s) -> ALL_RED (1s) -> SIDE_GREEN (8s max) -> SIDE_YELLOW (2s) -> ALL_RED (1s) -> MAIN_GREEN`.
    No conflicting vehicle greens; yellow clearance and all-red clearance strictly enforced.
  - Protected-turn traffic movement remains gated (`protectedTurnCommissioned = false`) pending physical junction geometry confirmation.

### 2.2 Pedestrian Signal Output Audit (4 Channels: `PED1`, `PED2` — `ULN2003A` Low-Side Driver)
- **Confirmed Physical Wiring**:
  - Only the 4 pedestrian lamp channels pass through the `ULN2003A` Darlington low-side driver:
    - **Pedestrian 1 (`PED1`)**: Walk = `GPIO 19`, Don't Walk = `GPIO 18`
    - **Pedestrian 2 (`PED2`)**: Walk = `GPIO 16`, Don't Walk = `GPIO 17`
- **Polarity (`OutputDriverType::LOW_SIDE_ULN2003A_ACTIVE_HIGH`)**:
  - Logical `ON` $\rightarrow$ ESP32 GPIO driven `HIGH` (`3.3V` at `ULN2003A` `INx`) $\rightarrow$ `ULN2003A` output transistor turns `ON` and pulls `OUTx` `LOW` to `GND` (`ulnSinkActive = true`) $\rightarrow$ common-anode pedestrian LED illuminates.
  - Logical `OFF` $\rightarrow$ ESP32 GPIO driven `LOW` (`0V` at `ULN2003A` `INx`) $\rightarrow$ `ULN2003A` output transistor `OFF` (`ulnSinkActive = false`) $\rightarrow$ pedestrian LED is `OFF`.
- **Mutual Exclusion**:
  - `WALK` and `DONT_WALK` on each pedestrian group are strictly mutually exclusive in all normal states and both `OFF` in `STARTUP_SAFE` and `FAULT_SAFE`.
- **Physical Commissioning Status**:
  - **CONFIRMED & COMMISSIONED**: All four physical lamps verified by operator on `ESP32-WROOM-32 30-Pin Board` (`COM5`).
  - Saved to ESP32 persistent NVS (`COMMISSION PED ON` + `SAVE`) and verified across multiple reboots.
  - Active configuration reports `Pedestrian Driver: LOW_SIDE_ULN2003A_ACTIVE_HIGH (Verified=YES)`.

---

## 3. Complete 21-Channel Pin-to-Pin Wiring Validation Table

| Ch / Signal | Locked ESP32 GPIO | Driver / Interface Stage | Physical Module Terminal | Active Polarity (`ESP32 -> Driver -> Lamp`) | Boot / Strap & Electrical Notes |
| :--- | :---: | :--- | :--- | :--- | :--- |
| **`MAIN_RED`** | **GPIO 27** | Direct ESP32 GPIO (`DIRECT_ACTIVE_HIGH`) | Traffic Light 1 (`MAIN`) `R` (`GND` to common `GND`) | `GPIO HIGH (3.3V)` -> **Lamp ON**; `GPIO LOW (0V)` -> **OFF** | Safe GP output; `GPIO LOW` at boot = OFF |
| **`MAIN_YELLOW`** | **GPIO 26** | Direct ESP32 GPIO (`DIRECT_ACTIVE_HIGH`) | Traffic Light 1 (`MAIN`) `Y` (`GND` to common `GND`) | `GPIO HIGH (3.3V)` -> **Lamp ON**; `GPIO LOW (0V)` -> **OFF** | Safe GP output; `GPIO LOW` at boot = OFF |
| **`MAIN_GREEN`** | **GPIO 25** | Direct ESP32 GPIO (`DIRECT_ACTIVE_HIGH`) | Traffic Light 1 (`MAIN`) `G` (`GND` to common `GND`) | `GPIO HIGH (3.3V)` -> **Lamp ON**; `GPIO LOW (0V)` -> **OFF** | Safe GP output; `GPIO LOW` at boot = OFF |
| **`SIDE_RED`** | **GPIO 14** | Direct ESP32 GPIO (`DIRECT_ACTIVE_HIGH`) | Traffic Light 2 (`SIDE`) `R` (`GND` to common `GND`) | `GPIO HIGH (3.3V)` -> **Lamp ON**; `GPIO LOW (0V)` -> **OFF** | Safe GP output; `GPIO LOW` at boot = OFF |
| **`SIDE_YELLOW`** | **GPIO 12*** | Direct ESP32 GPIO (`DIRECT_ACTIVE_HIGH`) | Traffic Light 2 (`SIDE`) `Y` (`GND` to common `GND`) | `GPIO HIGH (3.3V)` -> **Lamp ON**; `GPIO LOW (0V)` -> **OFF** | *`MTDI` strap: common-cathode LED holds `LOW` at reset (`3.3V` flash); never add pull-up on `GPIO 12` |
| **`SIDE_GREEN`** | **GPIO 13** | Direct ESP32 GPIO (`DIRECT_ACTIVE_HIGH`) | Traffic Light 2 (`SIDE`) `G` (`GND` to common `GND`) | `GPIO HIGH (3.3V)` -> **Lamp ON**; `GPIO LOW (0V)` -> **OFF** | Safe GP output; `GPIO LOW` at boot = OFF |
| **`TURN_RED`** | **GPIO 22** | Direct ESP32 GPIO (`DIRECT_ACTIVE_HIGH`) | Traffic Light 3 (`TURN`) `R` (`GND` to common `GND`) | `GPIO HIGH (3.3V)` -> **Lamp ON**; `GPIO LOW (0V)` -> **OFF** | Safe GP output; `GPIO LOW` at boot = OFF |
| **`TURN_YELLOW`** | **GPIO 23** | Direct ESP32 GPIO (`DIRECT_ACTIVE_HIGH`) | Traffic Light 3 (`TURN`) `Y` (`GND` to common `GND`) | `GPIO HIGH (3.3V)` -> **Lamp ON**; `GPIO LOW (0V)` -> **OFF** | Safe GP output; `GPIO LOW` at boot = OFF |
| **`TURN_GREEN`** | **GPIO 21** | Direct ESP32 GPIO (`DIRECT_ACTIVE_HIGH`) | Traffic Light 3 (`TURN`) `G` (`GND` to common `GND`) | `GPIO HIGH (3.3V)` -> **Lamp ON**; `GPIO LOW (0V)` -> **OFF** | Safe GP output; `GPIO LOW` at boot = OFF |
| **`PED1_WALK`** | **GPIO 19** | `ULN2003A` `IN1 -> OUT1` (`LOW_SIDE_ULN2003A_ACTIVE_HIGH`) | Ped 1 Green (`WALK`) cathode via series resistor (Anode to `+5V`) | `GPIO HIGH (3.3V)` -> `ULN2003A OUT1 LOW` -> **Lamp ON** | Common-anode `+5V`; `GPIO LOW` = OFF |
| **`PED1_DONT_WALK`** | **GPIO 18** | `ULN2003A` `IN2 -> OUT2` (`LOW_SIDE_ULN2003A_ACTIVE_HIGH`) | Ped 1 Red (`DONT_WALK`) cathode via series resistor (Anode to `+5V`) | `GPIO HIGH (3.3V)` -> `ULN2003A OUT2 LOW` -> **Lamp ON** | Common-anode `+5V`; `GPIO LOW` = OFF |
| **`PED2_WALK`** | **GPIO 16** | `ULN2003A` `IN3 -> OUT3` (`LOW_SIDE_ULN2003A_ACTIVE_HIGH`) | Ped 2 Green (`WALK`) cathode via series resistor (Anode to `+5V`) | `GPIO HIGH (3.3V)` -> `ULN2003A OUT3 LOW` -> **Lamp ON** | Common-anode `+5V`; `GPIO LOW` = OFF |
| **`PED2_DONT_WALK`** | **GPIO 17** | `ULN2003A` `IN4 -> OUT4` (`LOW_SIDE_ULN2003A_ACTIVE_HIGH`) | Ped 2 Red (`DONT_WALK`) cathode via series resistor (Anode to `+5V`) | `GPIO HIGH (3.3V)` -> `ULN2003A OUT4 LOW` -> **Lamp ON** | Common-anode `+5V`; `GPIO LOW` = OFF |
| **`PED1_BUTTON`** | **GPIO 32** | Direct to ESP32 (`INPUT_PULLUP`) | Ped Button 1 N.O. contact (other side to `GND`) | `Open = HIGH (3.3V)`, `Pressed = LOW (0V)` | 50 ms software debounce; live `RawHigh=1` verified |
| **`PED2_BUTTON`** | **GPIO 4*** | Direct to ESP32 (`INPUT_PULLUP`) | Ped Button 2 N.O. contact (other side to `GND`) | `Open = HIGH (3.3V)`, `Pressed = LOW (0V)` | *Strapping pin; N.O. button keeps `HIGH` at reset |
| **`IR1` (Main S1)** | **GPIO 33** | Direct to ESP32 (`INPUT_PULLUP`) | Main Road Sensor 1 `OUT` | `Idle = HIGH (3.3V)`, `Vehicle = LOW (0V)` | 100 ms stability filter; live `RawHigh=1` verified |
| **`IR2` (Main S2)** | **GPIO 34** | Direct to ESP32 (`INPUT` — input-only) | Main Road Sensor 2 `OUT` (with external $10\text{ k}\Omega$ pull-up to `3.3V`) | `Idle = HIGH (3.3V)`, `Vehicle = LOW (0V)` | Input-only (no internal pull-up); live `RawHigh=1` verified |
| **`IR3` (Side S1)** | **GPIO 35** | Direct to ESP32 (`INPUT` — input-only) | Side Road Sensor 1 `OUT` (with external $10\text{ k}\Omega$ pull-up to `3.3V`) | `Idle = HIGH (3.3V)`, `Vehicle = LOW (0V)` | Input-only (no internal pull-up); live `RawHigh=1` verified |
| **`IR4` (Side S2)** | **GPIO 36** | Direct to ESP32 (`INPUT` — input-only `VP`) | Side Road Sensor 2 `OUT` (with external $10\text{ k}\Omega$ pull-up to `3.3V`) | `Idle = HIGH (3.3V)`, `Vehicle = LOW (0V)` | Input-only (no internal pull-up); live `RawHigh=1` verified |
| **`IR5` (Turn S1)** | **GPIO 39** | Direct to ESP32 (`INPUT` — input-only `VN`) | Turn Road Sensor 1 `OUT` (with external $10\text{ k}\Omega$ pull-up to `3.3V`) | `Idle = HIGH (3.3V)`, `Vehicle = LOW (0V)` | Input-only (no internal pull-up); live `RawHigh=1` verified |
| **`IR6` (Turn S2)** | **GPIO 15*** | Direct to ESP32 (`INPUT_PULLUP`) | Turn Road Sensor 2 `OUT` | `Idle = HIGH (3.3V)`, `Vehicle = LOW (0V)` | *`MTDO` strap: idle `HIGH` = normal boot log; live `RawHigh=1` verified |

---

## 4. Controlled Serial CLI Commissioning & Single-Lamp Test Procedure

Once the physical checks in Sections 2 and 3 are confirmed with a multimeter, execute the following sequence over Serial (`COM5` @ `115200` baud):

```text
# 1. Enter MANUAL mode so the phase scheduler is held while testing individual lamps
MODE MANUAL

# 2. Commission Pedestrian ULN2003A Driver (after verifying Common-Anode +5V & series resistors)
DRIVER PED LOW_SIDE_ULN2003A
COMMISSION PED ON

# 3. Test each of the 4 pedestrian lamps individually (confirm all other 12 lamps stay OFF)
LAMP TEST PED1_W
LAMP TEST PED1_DW
LAMP TEST PED2_W
LAMP TEST PED2_DW
LAMP OFF

# 4. Commission Vehicle Direct Active-High Driver (after verifying series resistors & <=20mA @ 3.3V)
DRIVER VEH DIRECT_ACTIVE_HIGH
COMMISSION VEH ON

# 5. Test each of the 9 vehicle lamps individually (confirm all other 12 lamps stay OFF)
LAMP TEST MAIN_R
LAMP TEST MAIN_Y
LAMP TEST MAIN_G
LAMP TEST SIDE_R
LAMP TEST SIDE_Y
LAMP TEST SIDE_G
LAMP TEST TURN_R
LAMP TEST TURN_Y
LAMP TEST TURN_G
LAMP OFF

# 6. Trigger and release each physical IR sensor (IR1..IR6), verify Cycle=YES, then commission
SENSORS
COMMISSION IR ALL ON

# 7. (Optional) If physical RHT protected left-turn bay is present and verified
COMMISSION TURN ON

# 8. Return to autonomous AUTO control, save to NVS, and verify persistence across reboot
MODE AUTO
SAVE
REBOOT
STATUS
COMMISSION STATUS
```

---

## 5. IR Vehicle Detection Sensors Audit & Commissioning Verification

### 5.1 Six-Sensor Electrical & Hardware Audit Table

| Sensor ID | ESP32 GPIO | Assigned Movement | Approach Position | Input Hardware Configuration | Idle Electrical State | Active Polarity | Supervision & Safety Limits |
| :--- | :---: | :---: | :---: | :--- | :---: | :---: | :--- |
| **`IR1`** | **GPIO 33** | Main Road (`0`) | Upstream Approach (S1) | `INPUT_PULLUP` (Internal Pull-Up) | `RawHigh = 1` (`PASS`) | Active-`LOW` (Beam blocked = `LOW`) | 100 ms stability, 30 s stuck timeout, 12 trans/s chatter window |
| **`IR2`** | **GPIO 34** | Main Road (`0`) | Stopline / Presence (S2) | `INPUT` (Input-Only; External 10k Pull-Up to 3.3V) | `RawHigh = 1` (`PASS`) | Active-`LOW` (Beam blocked = `LOW`) | 100 ms stability, 30 s stuck timeout, 12 trans/s chatter window |
| **`IR3`** | **GPIO 35** | Side Road (`1`) | Upstream Approach (S1) | `INPUT` (Input-Only; External 10k Pull-Up to 3.3V) | `RawHigh = 1` (`PASS`) | Active-`LOW` (Beam blocked = `LOW`) | 100 ms stability, 30 s stuck timeout, 12 trans/s chatter window |
| **`IR4`** | **GPIO 36** | Side Road (`1`) | Stopline / Presence (S2) | `INPUT` (Input-Only `VP`; External 10k Pull-Up to 3.3V) | `RawHigh = 1` (`PASS`) | Active-`LOW` (Beam blocked = `LOW`) | 100 ms stability, 30 s stuck timeout, 12 trans/s chatter window |
| **`IR5`** | **GPIO 39** | Protected Turn (`2`) | Upstream Approach (S1) | `INPUT` (Input-Only `VN`; External 10k Pull-Up to 3.3V) | `RawHigh = 1` (`PASS`) | Active-`LOW` (Beam blocked = `LOW`) | 100 ms stability, 30 s stuck timeout, 12 trans/s chatter window |
| **`IR6`** | **GPIO 15** | Protected Turn (`2`) | Stopline / Presence (S2) | `INPUT_PULLUP` (`MTDO` boot strap pin) | `RawHigh = 1` (`PASS`) | Active-`LOW` (Beam blocked = `LOW`) | 100 ms stability, 30 s stuck timeout, 12 trans/s chatter window |

### 5.2 Filtering, Fault Isolation & Safety Invariants
1. **Debounce / Stability Window (`100 ms`)**:
   - Raw input must remain stable for $\ge 100\text{ ms}$ before updating `stableDetected`.
   - Prevents optical jitter or brief reflections from creating spurious vehicle arrivals.
2. **Stuck-Active Supervision (`30,000 ms`)**:
   - If an IR sensor remains continuously active for $> 30\text{ s}$, `stuckFault` is asserted, `FaultCode::SENSOR_STUCK_ACTIVE (0x0002)` is recorded, and the channel's latched demand is cleared.
   - Fault isolation ensures a stuck sensor never locks a green phase permanently.
   - Dual-sensor isolation: If Sensor 1 on an approach is stuck, Sensor 2 on that same approach continues to operate independently.
3. **Chatter / Floating Wire Supervision (`12 transitions/s`)**:
   - If a channel experiences $> 12$ transitions within a $1000\text{ ms}$ window, `chatterFault` is asserted, `FaultCode::SENSOR_DISCONNECTED_CHATTER (0x0004)` is recorded, and the input is suppressed until stable.
4. **Pre-Commissioning Health Gate (`CycleObserved = YES`)**:
   - Executing `COMMISSION IR <n> ON` requires:
     - `rawSampleCount > 0` (pin has been sampled).
     - `rawPinHigh == true` (pin is not held LOW).
     - `stuckFault == false` and `chatterFault == false`.
     - `cycleObserved == true` (a full, physical detect $\rightarrow$ release transition has occurred).
   - Prevents unverified, disconnected, or floating pins from being commissioned accidentally.

---

## 6. Outstanding Blockers & Commissioning Summary

| Component | Status | Remaining Operator Actions / Blockers |
| :--- | :---: | :--- |
| **Pedestrian Driver (`ULN2003A`)** | **COMMISSIONED** | None. Physical lamp illumination and mutual exclusion verified on target. |
| **Vehicle Driver (`DIRECT_ACTIVE_HIGH`)** | **COMMISSIONED** | None. All 9 lamps tested individually, clearance sequences, and reboot persistence verified. |
| **Protected Turn Movement (`TURN_GREEN`)** | **GATED / DISABLED** | Physical junction turning bay geometry and conflict visibility must be confirmed before `COMMISSION TURN ON`. |
| **6 × IR Vehicle Sensors (`IR1..IR6`)** | **UNCOMMISSIONED** | Physical detect-and-release sweep (`Cycle=YES`) must be performed before `COMMISSION IR ALL ON`. |
| **Live Traffic Readiness** | **HOLD** | Junction is NOT ready for live traffic until IR sensors and physical turning geometry are confirmed. |

---

## 7. Software Safety & Fault Architecture Verification

### 7.1 Fault Source & Response Matrix

| Fault Source | Code | Trigger Condition | Controller State | Output Response | Recovery Mechanism |
| :--- | :---: | :--- | :---: | :--- | :--- |
| **`OPERATOR_INJECTED_FAULT`** | `0x0200` | Serial CLI `FAULT` command | `FAULT_SAFE` | All 13 outputs driven `LOW (0V)` (Safe Electrical OFF) | Explicit `CLEAR` command $\rightarrow$ `STARTUP_SAFE (1s) -> ALL_RED (1s) -> MAIN_GREEN` |
| **`ILLEGAL_SIGNAL_COMBINATION`** | `0x0020` | Multiple greens, green during WALK, or aspect conflict | `FAULT_SAFE` | All 13 outputs driven `LOW (0V)` (Safe Electrical OFF) | Explicit `CLEAR` or `RESET` command |
| **`UNEXPECTED_STATE_TRANSITION`** | `0x0080` | Disallowed FSM transition attempted | `FAULT_SAFE` | All 13 outputs driven `LOW (0V)` (Safe Electrical OFF) | Explicit `CLEAR` or `RESET` command |
| **`UNCOMMISSIONED_TURN_CONFLICT`** | `0x0100` | Turn Green/Yellow active while turn uncommissioned | `FAULT_SAFE` | All 13 outputs driven `LOW (0V)` (Safe Electrical OFF) | Explicit `CLEAR` or `RESET` command |
| **`CONTROLLER_TIMING_FAULT`** | `0x0040` | Non-resting phase exceeds watchdog ($> 240\text{ s}$) | `FAULT_SAFE` | All 13 outputs driven `LOW (0V)` (Safe Electrical OFF) | Explicit `CLEAR` or `RESET` command |
| **`SENSOR_STUCK_ACTIVE`** | `0x0002` | IR sensor held LOW for $> 30\text{ s}$ | Normal FSM (`AUTO`) | Channel isolated; other sensor on approach remains active | Automatic on pin release (`rawPinHigh == true`) |
| **`SENSOR_DISCONNECTED_CHATTER`**| `0x0004` | IR sensor $> 12$ transitions / second | Normal FSM (`AUTO`) | Channel suppressed; demand cleared | Automatic after $\ge 100\text{ ms}$ stability |
| **`PED_BUTTON_STUCK_ACTIVE`** | `0x0008` | Ped button held LOW for $> 30\text{ s}$ | Normal FSM (`AUTO`) | Single request served; subsequent holds ignored | Automatic on button release (`rawPinHigh == true`) |
| **`INVALID_CONFIGURATION`** | `0x0010` | Corrupted NVS checksum or out-of-range timings | Normal FSM (`AUTO`) | Sanitized to documented safe defaults; no FSM crash | Automatic fallback to defaults |
| **`INVALID_SENSOR_STATE`** | `0x0001` | Corrupted adapter frame | Normal FSM (`AUTO`) | Corrupted frame discarded; channel states cleared | Next valid frame |

### 7.2 Safety Interlocks & Transition Constraints
1. **Clearance Rejection Gate:** In `MODE MANUAL`, issuing `NEXT` during `MAIN_YELLOW`, `SIDE_YELLOW`, `TURN_YELLOW`, or `ALL_RED` is rejected with `Clearance transition in progress; wait for green/walk phase`. Clearance intervals can never be skipped or bypassed.
2. **Pedestrian Group Isolation:** When only `PED1` is requested, during `PED_WALK` `PED1` displays `WALK` while `PED2` strictly displays `DONT_WALK`. All vehicular approaches display `RED`. A configurable post-walk clearance (`pedClearanceMs`) in `ALL_RED` is enforced before vehicle green resumes.
3. **Protected Turn Multi-Layer Gate:** Protected turn demand is ignored while `protectedTurnCommissioned == false`. FSM target selection, clearance transitions, conflict matrix, and update watchdog all independently enforce this gate.

---

## 8. Automatic Fixed-Time Sequencing & Sensor Isolation Commissioning

### 8.1 Confirmed Prototype Fixed-Time Timings

| Step | Phase | Prototype Duration | Measured on ESP32 (`COM5`) | Status | Notes |
| :---: | :--- | :---: | :---: | :---: | :--- |
| **0** | `STARTUP_SAFE` | $1,000\text{ ms}$ | $1,000\text{ ms}$ | **PASS** | Safe electrical off before initial clearance |
| **0.5**| Initial `ALL_RED` | $2,000\text{ ms}$ | $2,000\text{ ms}$ | **PASS** | Startup clearance before main arterial green |
| **1** | **`MAIN_GREEN`** | **$25,000\text{ ms}$ ($25\text{ s}$)** | **$25,000\text{ ms}$ ($25.00\text{ s}$)** | **PASS** | Fixed arterial green; immune to uncommissioned IR noise |
| **2** | **`MAIN_YELLOW`**| **$3,000\text{ ms}$ ($3\text{ s}$)** | **$3,000\text{ ms}$ ($3.00\text{ s}$)** | **PASS** | Yellow change interval |
| **3** | **`ALL_RED`** | **$2,000\text{ ms}$ ($2\text{ s}$)** | **$2,000\text{ ms}$ ($2.00\text{ s}$)** | **PASS** | Intersection clearance interval |
| **4** | **`SIDE_GREEN`** | **$20,000\text{ ms}$ ($20\text{ s}$)** | **$20,000\text{ ms}$ ($20.00\text{ s}$)** | **PASS** | Fixed side-street green; holds full 20s without early termination |
| **5** | **`SIDE_YELLOW`**| **$3,000\text{ ms}$ ($3\text{ s}$)** | **$3,000\text{ ms}$ ($3.00\text{ s}$)** | **PASS** | Yellow change interval |
| **6** | **`ALL_RED`** | **$2,000\text{ ms}$ ($2\text{ s}$)** | **$2,000\text{ ms}$ ($2.00\text{ s}$)** | **PASS** | Intersection clearance interval |
| **Cycle**| Continuous Repeat | Full cycle: $55\text{ s}$ | $55.00\text{ s}$ | **PASS** | Repeats continuously |

### 8.2 Sensor Isolation Verification
- **Uncommissioned IR Sensors Isolated:** All 6 IR channels report `Validated=NO` (`sensorValidatedMask = 0x00`).
- **No Early Termination / Extension:** During `MAIN_GREEN` and `SIDE_GREEN`, uncommissioned IR inputs do not trigger phase extension, early green termination, or state jumps.
- **Safety Mode Gating:** Command `MODE ADAPTIVE` is strictly rejected:
  `[EVENT: COMMAND_REJECTED | MODE ADAPTIVE unavailable: physical IR sensors must pass commissioning first]`.

### 8.3 Pedestrian Coordination & Coordinated Insertion
- **Insertion Point:** Pedestrian demand latched during fixed-time cycle is safely served following Side Road clearance.
- **Mutual Exclusion:** Vehicles held at all-red (`MAIN_R=1`, `SIDE_R=1`, `TURN_R=1`) throughout `PED_WALK` ($5,000\text{ ms}$).
- **Post-Walk Clearance:** Post-WALK `ALL_RED` clearance ($2,000\text{ ms}$) enforced before arterial `MAIN_GREEN` resumes.
- **Group Isolation:** Single crossing call (e.g. `PED 1`) activates only `PED1_WALK` while `PED2` strictly displays `DONT_WALK`.

### 8.4 Target Verification & Firmware Delivery
- **Unit Test Suite:** All **49 / 49** Unity test cases pass directly on ESP32 target `COM5` in $50.18\text{ s}$.
- **Production Firmware:** Uploaded at $921,600\text{ baud}$ and verified on `ESP32-WROOM-32 30-Pin Board` (`COM5`). Boot mode defaults to `MODE FIXED` with prototype timings persisted in NVS across reboots.

---

## 9. Final Commissioning, Sensor Validation Matrix & Physical Actions

### 9.1 Verification Summary Table

| Subsystem / Item | Verification Status | Verification Detail / Evidence |
| :--- | :---: | :--- |
| **Firmware Build & Upload** | **PASS** | PlatformIO release build compiled cleanly; flashed to ESP32 on `COM5` at $921,600\text{ baud}$ with SHA digest verification and hard reset. |
| **Automated Unit Tests** | **PASS** | **49 / 49** Unity test cases succeeded on ESP32 target `COM5` in $50.18\text{ s}$. |
| **Vehicle Traffic Lights (9 Lamps)** | **PASS** | All 9 lamps (`MAIN` 27/26/25, `SIDE` 13/12/14, `TURN` 23/22/21) verified individually, mutual exclusion confirmed, persisted in NVS (`DIRECT_ACTIVE_HIGH`, `Verified=YES`). |
| **Pedestrian Lights (4 Lamps)** | **PASS** | All 4 lamps (`PED1` 19/18, `PED2` 17/16) verified via ULN2003A low-side sink, mutual exclusion confirmed, persisted in NVS (`LOW_SIDE_ULN2003A_ACTIVE_HIGH`, `Verified=YES`). |
| **Pedestrian Pushbuttons (2 Inputs)** | **PASS** | GPIO 32 (`PED1_BTN`) and GPIO 4 (`PED2_BTN`) sampled idle HIGH (`RawHigh=1`), internal pull-ups verified, debounced ($50\text{ ms}$) and isolated. |
| **Fixed-Time Sequencing** | **PASS** | Exact prototype sequence executed live: Main Green $25\text{ s}$, Yellow $3\text{ s}$, All-Red $2\text{ s}$, Side Green $20\text{ s}$, Yellow $3\text{ s}$, All-Red $2\text{ s}$ ($55\text{ s}$ total cycle). |
| **Adaptive Demand Scheduler** | **PASS** | Demand scoring, starvation prevention ($+25$ after 2 skips), waiting-time weights ($2\text{ s}$), and safe phase boundary transitions verified in Test 18, 19, 43, 49. |
| **Sensor Fault Fallback Recall** | **PASS** | If an IR sensor faults or chatters, adaptive mode automatically detects the condition and falls back to fixed-time recall (Test 49). |
| **Protected Turn Safety Gate** | **PASS** | Remains `UNCOMMISSIONED_DISABLED` pending physical junction geometry confirmation; turn displays continuous `RED`. |
| **IR1 (GPIO 33 - Main Road S1)** | **NOT TESTED** | Idle electrical check `PASS` (`RawHigh=1`). Awaiting physical obstacle detect-and-release sweep (`Cycle=YES`). |
| **IR2 (GPIO 34 - Main Road S2)** | **NOT TESTED** | Idle electrical check `PASS` (`RawHigh=1`). Awaiting physical obstacle detect-and-release sweep (`Cycle=YES`). |
| **IR3 (GPIO 35 - Side Road S1)** | **NOT TESTED** | Idle electrical check `PASS` (`RawHigh=1`). Awaiting physical obstacle detect-and-release sweep (`Cycle=YES`). |
| **IR4 (GPIO 36 - Side Road S2)** | **NOT TESTED** | Idle electrical check `PASS` (`RawHigh=1`). Awaiting physical obstacle detect-and-release sweep (`Cycle=YES`). |
| **IR5 (GPIO 39 - Turn Road S1)** | **NOT TESTED** | Idle electrical check `PASS` (`RawHigh=1`). Awaiting physical obstacle detect-and-release sweep (`Cycle=YES`). |
| **IR6 (GPIO 15 - Turn Road S2)** | **NOT TESTED** | Idle electrical check `PASS` (`RawHigh=1`). Awaiting physical obstacle detect-and-release sweep (`Cycle=YES`). |

### 9.2 Operator Actions for Physical Sensor Obstacle Sweep & Validation

All 6 IR sensors have passed the idle electrical check (`RawHigh=1`, idle unblocked). To complete final commissioning:

1. **Perform Physical Sweep (IR1 to IR6):**
   - Connect to ESP32 on `COM5` via serial monitor at 115200 baud.
   - Sweep an obstacle (e.g., hand or test vehicle) in front of each sensor in turn.
   - Observe live event feedback: `[EVENT: SENSOR_DETECTED]` followed by `[EVENT: SENSOR_RELEASED]`.
   - Run command `SENSORS` or `COMMISSION STATUS` to verify `Cycle=YES` and `Ready=YES` for that channel.
   - Execute `COMMISSION SENSOR <1-6> ON` and `SAVE` to persist validation in NVS.
   - Once all 6 sensors report `Validated=YES`, enable adaptive mode via command `MODE ADAPTIVE`.

2. **Execute Physical Detect-and-Release Sweep (Sensors 1 to 6):**
   - Open serial terminal (`COM5` at $115,200\text{ baud}$).
   - Pass a vehicle model or hand in front of each sensor in sequence:
     - **IR1 (GPIO 33)** $\rightarrow$ Place object ($\ge 100\text{ ms}$) $\rightarrow$ remove object.
     - **IR2 (GPIO 34)** $\rightarrow$ Place object ($\ge 100\text{ ms}$) $\rightarrow$ remove object.
     - **IR3 (GPIO 35)** $\rightarrow$ Place object ($\ge 100\text{ ms}$) $\rightarrow$ remove object.
     - **IR4 (GPIO 36)** $\rightarrow$ Place object ($\ge 100\text{ ms}$) $\rightarrow$ remove object.
     - **IR5 (GPIO 39)** $\rightarrow$ Place object ($\ge 100\text{ ms}$) $\rightarrow$ remove object.
     - **IR6 (GPIO 15)** $\rightarrow$ Place object ($\ge 100\text{ ms}$) $\rightarrow$ remove object.
   - Observe serial event: `[EVENT: SENSOR_RELEASED | IRx (GPIO yy) released (Cycle=YES, Ready to Commission)]`.

3. **Commission and Enable Adaptive Mode:**
   - Execute via serial CLI:
     ```text
     COMMISSION IR ALL ON
     MODE ADAPTIVE
     SAVE
     ```
   - All 6 sensors will report `Validated=YES` and the controller will transition seamlessly into full adaptive vehicle-actuated operation.





