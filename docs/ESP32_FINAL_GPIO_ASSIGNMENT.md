# ESP32 Final GPIO Assignment & Electrical Driver Audit

**Target Board:** ESP32-WROOM-32 DevKit V1 (38-Pin)  
**Source Authority:** `include/TrafficConfig.h` and `include/SignalOutputDriver.h`

---

## 1. Critical Electrical Compatibility Findings

### 1.1 Vehicle RYG Signal Modules (5 V Common-Ground) — **BLOCKED PENDING HIGH-SIDE DRIVER**
- **Measured Module Polarity**: Each of the three 4-wire vehicle RYG modules (`GND`, `R`, `Y`, `G`) has a **common ground (`GND`)** and illuminates when **`+5V` is sourced** into the `R`, `Y`, or `G` input wire.
- **Why `ULN2003A` Cannot Drive Vehicle Modules Directly**:
  - The `ULN2003A` is an array of 7 NPN Darlington **low-side current sinks** (open-collector to `GND`).
  - It can only sink current to `GND`; it **cannot source `+5V`** to a common-ground load.
  - **Rule**: Do **not** connect the 5 V common-ground vehicle modules directly to `ULN2003A` outputs.
- **Required Hardware Solution for Vehicle Modules**:
  - Use a high-side `+5V` sourcing driver stage per channel (e.g., `UDN2981A` / `TBD62783APG` 8-channel high-side source driver, or `ULN2003A` stage driving PNP/P-MOSFET high-side switches, or high-side pull-up load stage if verified within current limits).
  - Until the high-side driver hardware is verified, `TrafficConfig::VEHICLE_HIGH_SIDE_DRIVER_VERIFIED` remains `false` and `SignalOutputDriver` keeps vehicle channels de-energized in `UNVERIFIED_SAFE_DISABLED` mode.

### 1.2 Pedestrian RED/GREEN Signal Groups — **COMPATIBLE WITH `ULN2003A`**
- The two pedestrian signal pairs consist of discrete RED and GREEN LEDs with accessible anodes and cathodes.
- They may be driven by a `ULN2003A` IC using **common-anode (`+5V`)** wiring with individual current-limiting resistors ($330\text{ }\Omega$ to $470\text{ }\Omega$ per LED) between each LED cathode and the `ULN2003A` `OUTx` terminal.

---

## 2. Complete 21-Channel GPIO Allocation Table

All existing `v1.1.0` pin assignments are preserved by default so existing wiring is not silently broken.

| Group | Signal / Sensor | Firmware Constant | Default GPIO | Proposed Safe Rewire GPIO | Direction / Pull Mode | Electrical Notes |
| :--- | :--- | :--- | :---: | :---: | :--- | :--- |
| **Vehicle 1 (`MAIN`)** | Main Green | `PIN_MAIN_GREEN` | **GPIO 27** | GPIO 27 | `OUTPUT` | Safe GP output; requires high-side +5V driver |
| **Vehicle 1 (`MAIN`)** | Main Yellow | `PIN_MAIN_YELLOW` | **GPIO 26** | GPIO 26 | `OUTPUT` | Safe GP output; requires high-side +5V driver |
| **Vehicle 1 (`MAIN`)** | Main Red | `PIN_MAIN_RED` | **GPIO 25** | GPIO 25 | `OUTPUT` | Safe GP output; requires high-side +5V driver |
| **Vehicle 2 (`SIDE`)** | Side Green | `PIN_SIDE_GREEN` | **GPIO 13** | GPIO 13 | `OUTPUT` | Safe GP output; requires high-side +5V driver |
| **Vehicle 2 (`SIDE`)** | Side Yellow | `PIN_SIDE_YELLOW` | **GPIO 12*** | **GPIO 5** (`PIN_SIDE_YELLOW_SAFE_PROPOSED`) | `OUTPUT` | *`GPIO 12` is `MTDI` flash-voltage strap (see Section 3) |
| **Vehicle 2 (`SIDE`)** | Side Red | `PIN_SIDE_RED` | **GPIO 14** | GPIO 14 | `OUTPUT` | Safe GP output; requires high-side +5V driver |
| **Vehicle 3 (`TURN`)** | Turn Green | `PIN_TURN_GREEN` | **GPIO 23** | GPIO 23 | `OUTPUT` | Safe GP output; requires high-side +5V driver |
| **Vehicle 3 (`TURN`)** | Turn Yellow | `PIN_TURN_YELLOW` | **GPIO 22** | GPIO 22 | `OUTPUT` | Safe GP output; requires high-side +5V driver |
| **Vehicle 3 (`TURN`)** | Turn Red | `PIN_TURN_RED` | **GPIO 21** | GPIO 21 | `OUTPUT` | Safe GP output; requires high-side +5V driver |
| **Pedestrian 1 (`PED1`)** | Ped 1 GREEN (`WALK`) | `PIN_PED1_WALK` | **GPIO 19** | GPIO 19 | `OUTPUT` | Compatible with `ULN2003A` (Common-Anode LED) |
| **Pedestrian 1 (`PED1`)** | Ped 1 RED (`DONT_WALK`) | `PIN_PED1_DONT_WALK` | **GPIO 18** | GPIO 18 | `OUTPUT` | Compatible with `ULN2003A` (Common-Anode LED) |
| **Pedestrian 2 (`PED2`)** | Ped 2 GREEN (`WALK`) | `PIN_PED2_WALK` | **GPIO 17** | GPIO 17 | `OUTPUT` | Compatible with `ULN2003A` (Common-Anode LED) |
| **Pedestrian 2 (`PED2`)** | Ped 2 RED (`DONT_WALK`) | `PIN_PED2_DONT_WALK` | **GPIO 16** | GPIO 16 | `OUTPUT` | Compatible with `ULN2003A` (Common-Anode LED) |
| **Ped Button 1** | Ped 1 Pushbutton | `PIN_PED1_BUTTON` | **GPIO 32** | GPIO 32 | `INPUT_PULLUP` | Active-LOW switch to `GND` (50 ms debounce) |
| **Ped Button 2** | Ped 2 Pushbutton | `PIN_PED2_BUTTON` | **GPIO 4** | GPIO 4 | `INPUT_PULLUP` | Active-LOW switch to `GND` (50 ms debounce) |
| **IR Sensor 1** | Side Road Primary | `PIN_IR_SENSOR_1` | **GPIO 33** | GPIO 33 | `INPUT_PULLUP` | Active-LOW, 3.3 V logic level required |
| **IR Sensor 2** | Side Road Secondary | `PIN_IR_SENSOR_2` | **GPIO 34** | GPIO 34 | `INPUT` (Input-Only) | **Requires external $10\text{ k}\Omega$ pull-up to 3.3 V** |
| **IR Sensor 3** | Main Road Primary | `PIN_IR_SENSOR_3` | **GPIO 35** | GPIO 35 | `INPUT` (Input-Only) | **Requires external $10\text{ k}\Omega$ pull-up to 3.3 V** |
| **IR Sensor 4** | Main Road Secondary | `PIN_IR_SENSOR_4` | **GPIO 36** (`VP`) | GPIO 36 | `INPUT` (Input-Only) | **Requires external $10\text{ k}\Omega$ pull-up to 3.3 V** |
| **IR Sensor 5** | Protected Turn Primary | `PIN_IR_SENSOR_5` | **GPIO 39** (`VN`) | GPIO 39 | `INPUT` (Input-Only) | **Requires external $10\text{ k}\Omega$ pull-up to 3.3 V** |
| **IR Sensor 6** | Protected Turn Secondary | `PIN_IR_SENSOR_6` | **GPIO 15** | GPIO 15 | `INPUT_PULLUP` | `MTDO` strapping pin; idle Active-LOW sensor holds `HIGH` |

---

## 3. Boot-Strapping & Restricted Pin Audit

1. **`GPIO 6, 7, 8, 9, 10, 11` (Internal SPI Flash)**: Strictly forbidden and unused.
2. **`GPIO 1 (TX0), GPIO 3 (RX0)` (USB Serial UART0)**: Reserved exclusively for serial diagnostics and telemetry.
3. **`GPIO 0` & `GPIO 2` (Bootloader Strapping Pins)**: Unused to prevent serial boot/flashing lockups.
4. **`GPIO 12` (`MTDI` Flash Voltage Strapping Pin)**:
   - Retained as `PIN_SIDE_YELLOW = 12` by default only to avoid silently changing existing `v1.1.0` wiring.
   - **Risk**: If an external circuit pulls `GPIO 12` `HIGH` during reset, the ESP32 selects 1.8 V for `VDD_SDIO` and fails to boot from its 3.3 V SPI flash.
   - **Recommended Rewiring**: Move the Side Yellow signal wire from `GPIO 12` to **`GPIO 5`** (`PIN_SIDE_YELLOW_SAFE_PROPOSED = 5`) and call `setUseSafeProposedSideYellowPin(true)`.
5. **`GPIO 34, 35, 36, 39` (Input-Only GPIOs)**:
   - Assigned strictly to `IR2`, `IR3`, `IR4`, and `IR5` inputs.
   - Because ESP32 silicon omits internal pull-ups on `GPIO 34–39`, external $10\text{ k}\Omega$ pull-up resistors to `3.3 V` are mandatory before enabling those sensor channels.

