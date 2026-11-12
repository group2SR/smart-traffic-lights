# ESP32-WROOM-32 30-Pin Final GPIO Assignment & Hardware Audit

**Target Board:** ESP32-WROOM-32 Development Board (30-Pin DIP Form Factor / DOIT DevKit V1 30-Pin)  
**Silicon Module:** ESP32-WROOM-32 (ESP32-D0WDQ6, 240 MHz Dual-Core Xtensa LX6, 520 KB SRAM, 4 MB SPI Flash)  
**Source Authority:** `include/TrafficConfig.h` and `include/SignalOutputDriver.h`  
**Serial Interface:** USB Serial UART0 on `COM5` (115200 Baud, 8N1)

---

## 1. 30-Pin Physical Board Architecture & Pinout Map

The controller is an **ESP32-WROOM-32 30-pin development board** featuring two 15-pin single-row headers ($2.54\text{ mm}$ pitch, standard $0.9\text{ inch}$ DIP breadboard spacing).

```
                             +------------------------+
                             |       ESP-WROOM-32     |
                             |       PCB ANTENNA      |
                             |  +------------------+  |
                             |  |   ESP32-D0WDQ6   |  |
                             |  +------------------+  |
                             +------------------------+
           LEFT HEADER (J1)                              RIGHT HEADER (J2)
   Pin  Silk    GPIO   Function                 Function   GPIO   Silk   Pin
  +----+-------+------+-----------------------+----------+------+-------+----+
  |  1 | EN    | CHIP | Enable / Reset Button | V_SPI_D  | GPIO23| D23   |  1 | (Turn Red)
  |  2 | VP    | GPI36| IR4: Side Rear S2     | I2C SCL  | GPIO22| D22   |  2 | (Turn Yellow)
  |  3 | VN    | GPI39| IR5: Turn Front S1    | UART0 TX | GPIO1 | TX0   |  3 | (USB Telemetry)
  |  4 | D34   | GPI34| IR2: Main Rear S2     | UART0 RX | GPIO3 | RX0   |  4 | (USB Commands)
  |  5 | D35   | GPI35| IR3: Side Front S1    | I2C SDA  | GPIO21| D21   |  5 | (Turn Green)
  |  6 | D32   | GPIO32| PED1 Pushbutton      | V_SPI_Q  | GPIO19| D19   |  6 | (Ped 1 Walk)
  |  7 | D33   | GPIO33| IR1: Main Front S1   | V_SPI_CLK| GPIO18| D18   |  7 | (Ped 1 Don't Walk)
  |  8 | D25   | GPIO25| Main Green           | Unused   | GPIO5 | D5    |  8 | (Spare)
  |  9 | D26   | GPIO26| Main Yellow          | U2TXD    | GPIO17| TX2   |  9 | (Ped 2 Walk)
  | 10 | D27   | GPIO27| Main Red             | U2RXD    | GPIO16| RX2   | 10 | (Ped 2 Don't Walk)
  | 11 | D14   | GPIO14| Side Red             | Touch 0  | GPIO4 | D4    | 11 | (PED2 Pushbutton)
  | 12 | D12   | GPIO12| Side Green (MTDI)    | Boot/LED | GPIO2 | D2    | 12 | (Spare / Strap)
  | 13 | D13   | GPIO13| Side Yellow          | MTDO     | GPIO15| D15   | 13 | (IR6: Turn Rear S2)
  | 14 | GND   | GND  | Ground Rail           | Ground   | GND   | GND   | 14 | (Ground Rail)
  | 15 | VIN   | 5V   | External +5V Power    | Reg Out  | 3.3V  | 3V3   | 15 | (3.3V LDO Rail)
  +----+-------+------+-----------------------+----------+------+-------+----+
                             |      [Micro-USB]       |
                             +------------------------+
```

---

## 2. 30-Pin Physical Pin Availability & Channel Allocation

Every single one of the 21 traffic-control channels and 2 USB serial channels is physically exposed and available on the 30-pin board header:

| Subsystem | Required Channels | Allocated GPIOs | Physical 30-Pin Positions | Availability Verification |
| :--- | :---: | :--- | :--- | :---: |
| **Main Road Traffic Light** | 3 | Red=27, Yel=26, Grn=25 | Left Header Pins 10, 9, 8 | **CONFIRMED** |
| **Side Road Traffic Light** | 3 | Red=14, Yel=13, Grn=12 | Left Header Pins 11, 13, 12 | **CONFIRMED** |
| **Turn Road Traffic Light** | 3 | Red=23, Yel=22, Grn=21 | Right Header Pins 1, 2, 5 | **CONFIRMED** |
| **Pedestrian 1 Signal** | 2 | Walk=19, Don't Walk=18 | Right Header Pins 6, 7 | **CONFIRMED** |
| **Pedestrian 2 Signal** | 2 | Walk=17, Don't Walk=16 | Right Header Pins 9 (`TX2`), 10 (`RX2`) | **CONFIRMED** |
| **Pedestrian Pushbuttons** | 2 | Button 1=32, Button 2=4 | Left Pin 6 (`D32`), Right Pin 11 (`D4`) | **CONFIRMED** |
| **Main Road IR Sensors** | 2 | IR1=33, IR2=34 | Left Pin 7 (`D33`), Left Pin 4 (`D34`) | **CONFIRMED** |
| **Side Road IR Sensors** | 2 | IR3=35, IR4=36 | Left Pin 5 (`D35`), Left Pin 2 (`VP`) | **CONFIRMED** |
| **Turn Road IR Sensors** | 2 | IR5=39, IR6=15 | Left Pin 3 (`VN`), Right Pin 13 (`D15`) | **CONFIRMED** |
| **Serial Diagnostics UART0**| 2 | TX=1, RX=3 | Right Pin 3 (`TX0`), Right Pin 4 (`RX0`) | **CONFIRMED** |
| **Spare / Unused Pins** | 2 | GPIO 5, GPIO 2 | Right Pin 8 (`D5`), Right Pin 12 (`D2`) | *Available for expansion* |
| **Power & Ground Rails** | 4 | GND (x2), 3V3 (x1), VIN (x1)| Left 14/15, Right 14/15 | **CONFIRMED** |
| **Total Pins Accounted For**| **30** | — | **All 30 Header Pins Mapped** | **100% MATCH** |

---

## 3. Hardware Constraint & Silicon Boundary Verifications

### 3.1 Input-Only Restrictions on GPIO 34, 35, 36 (`VP`), and 39 (`VN`)
- **Silicon Rule**: Pins `GPI 34`, `GPI 35`, `GPI 36` (`SENSOR_VP`), and `GPI 39` (`SENSOR_VN`) on ESP32 silicon are **strictly input-only**. They do **not** possess internal output drivers or internal pull-up / pull-down silicon resistors.
- **Calling `pinMode(pin, INPUT_PULLUP)`** on these pins has no physical effect in silicon (they remain floating high-impedance inputs).
- **Assigned Channels**:
  - `GPIO 34` (`D34`, Left Pin 4) $\rightarrow$ `IR2` (Main Road Rear Sensor S2)
  - `GPIO 35` (`D35`, Left Pin 5) $\rightarrow$ `IR3` (Side Road Front Sensor S1)
  - `GPIO 36` (`VP`, Left Pin 2)  $\rightarrow$ `IR4` (Side Road Rear Sensor S2)
  - `GPIO 39` (`VN`, Left Pin 3)  $\rightarrow$ `IR5` (Turn Road Front Sensor S1)
- **Mandatory Hardware Requirement**: External **$10\text{ k}\Omega$ pull-up resistors to $3.3\text{ V}$** must be installed on each of these 4 pins between the sensor signal terminal and the ESP32 $3.3\text{ V}$ rail.

### 3.2 GPIO 16 (`RX2`) and GPIO 17 (`TX2`) Availability
- **30-Pin Exposure**: On the 30-pin board, GPIO 16 is physically brought out to Right Pin 10 (silk-screened `RX2`), and GPIO 17 is physically brought out to Right Pin 9 (silk-screened `TX2`).
- **Internal Flash / PSRAM Verification**: Standard ESP32-WROOM-32 modules (without external SPI PSRAM) do not use GPIO 16/17 for flash memory. The integrated SPI flash is connected exclusively to internal pins GPIO 6–11 (which are not broken out on the 30-pin board header, eliminating any accidental contention risk).
- **Assigned Channels**:
  - `GPIO 17` (`TX2`, Right Pin 9)  $\rightarrow$ `PIN_PED2_WALK` (Pedestrian 2 Walk LED via ULN2003A)
  - `GPIO 16` (`RX2`, Right Pin 10) $\rightarrow$ `PIN_PED2_DONT_WALK` (Pedestrian 2 Don't Walk LED via ULN2003A)
- **Status**: Both pins are verified fully functional as push-pull digital outputs driving the ULN2003A driver.

### 3.3 Boot-Strapping Pin Implications: GPIO 12 (`MTDI`) and GPIO 15 (`MTDO`)
- **GPIO 12 (`MTDI`, Left Pin 12)**:
  - **Function**: Boot-strapping pin for internal flash voltage LDO (`VDD_SDIO`).
  - **Behavior**: If sampled `HIGH` ($3.3\text{ V}$) at power-on reset, the ESP32 selects $1.8\text{ V}$ for SPI flash power, causing immediate boot failure from $3.3\text{ V}$ flash. If sampled `LOW` ($0\text{ V}$), it selects $3.3\text{ V}$ flash operation.
  - **Project Circuit**: Connected to `PIN_SIDE_GREEN = 12`. The Side Green LED anode is tied to GPIO 12 with its cathode returning to circuit ground. During bootloader sampling, the unenergized LED and series resistor naturally hold GPIO 12 at $0\text{ V}$ (LOW), guaranteeing normal 3.3V boot.
  - **Safety Warning**: **Never place an external pull-up resistor on GPIO 12.**
- **GPIO 15 (`MTDO`, Right Pin 13)**:
  - **Function**: Boot-strapping pin for ROM bootloader UART logging.
  - **Behavior**: If sampled `HIGH` at reset, normal ROM bootloader logging to UART0 is enabled. If sampled `LOW`, boot logging is muted (silent boot).
  - **Project Circuit**: Connected to `PIN_TURN_IR_SENSOR_2 = 15` configured with `INPUT_PULLUP`. Because the IR sensor is active-LOW, in its idle state (no vehicle detected) its output is held `HIGH` ($3.3\text{ V}$), ensuring normal bootloader logging and clean system initialization.

---

## 4. Complete 21-Channel Locked GPIO Allocation Table

All 21 physical signal assignments are locked and defined in `include/TrafficConfig.h`.

| Group | Signal / Sensor Channel | Firmware Constant | Locked GPIO | 30-Pin Board Pin | Direction & Mode | Electrical Topology & Notes |
| :--- | :--- | :--- | :---: | :---: | :--- | :--- |
| **Vehicle 1 (`MAIN`)** | Main Red | `PIN_MAIN_RED` | **GPIO 27** | **Left Pin 10** (`D27`) | `OUTPUT` | Active-HIGH; requires +5V high-side sourcing driver |
| **Vehicle 1 (`MAIN`)** | Main Yellow | `PIN_MAIN_YELLOW` | **GPIO 26** | **Left Pin 9** (`D26`) | `OUTPUT` | Active-HIGH; requires +5V high-side sourcing driver |
| **Vehicle 1 (`MAIN`)** | Main Green | `PIN_MAIN_GREEN` | **GPIO 25** | **Left Pin 8** (`D25`) | `OUTPUT` | Active-HIGH; requires +5V high-side sourcing driver |
| **Vehicle 2 (`SIDE`)** | Side Red | `PIN_SIDE_RED` | **GPIO 14** | **Left Pin 11** (`D14`) | `OUTPUT` | Active-HIGH; requires +5V high-side sourcing driver |
| **Vehicle 2 (`SIDE`)** | Side Yellow | `PIN_SIDE_YELLOW` | **GPIO 13** | **Left Pin 13** (`D13`) | `OUTPUT` | Active-HIGH; requires +5V high-side sourcing driver |
| **Vehicle 2 (`SIDE`)** | Side Green | `PIN_SIDE_GREEN` | **GPIO 12** | **Left Pin 12** (`D12`) | `OUTPUT` | MTDI boot-strap pin; held LOW at reset by LED |
| **Vehicle 3 (`TURN`)** | Turn Red | `PIN_TURN_RED` | **GPIO 23** | **Right Pin 1** (`D23`) | `OUTPUT` | Active-HIGH; requires +5V high-side sourcing driver |
| **Vehicle 3 (`TURN`)** | Turn Yellow | `PIN_TURN_YELLOW` | **GPIO 22** | **Right Pin 2** (`D22`) | `OUTPUT` | Active-HIGH; requires +5V high-side sourcing driver |
| **Vehicle 3 (`TURN`)** | Turn Green | `PIN_TURN_GREEN` | **GPIO 21** | **Right Pin 5** (`D21`) | `OUTPUT` | Active-HIGH; requires +5V high-side sourcing driver |
| **Pedestrian 1 (`PED1`)**| Ped 1 GREEN (`WALK`) | `PIN_PED1_WALK` | **GPIO 19** | **Right Pin 6** (`D19`) | `OUTPUT` | Compatible with `ULN2003A` low-side current sink |
| **Pedestrian 1 (`PED1`)**| Ped 1 RED (`DONT_WALK`)| `PIN_PED1_DONT_WALK`| **GPIO 18** | **Right Pin 7** (`D18`) | `OUTPUT` | Compatible with `ULN2003A` low-side current sink |
| **Pedestrian 2 (`PED2`)**| Ped 2 GREEN (`WALK`) | `PIN_PED2_WALK` | **GPIO 17** | **Right Pin 9** (`TX2`) | `OUTPUT` | Compatible with `ULN2003A` low-side current sink |
| **Pedestrian 2 (`PED2`)**| Ped 2 RED (`DONT_WALK`)| `PIN_PED2_DONT_WALK`| **GPIO 16** | **Right Pin 10** (`RX2`)| `OUTPUT` | Compatible with `ULN2003A` low-side current sink |
| **Ped Button 1** | Ped 1 Pushbutton | `PIN_PED1_BUTTON` | **GPIO 32** | **Left Pin 6** (`D32`) | `INPUT_PULLUP` | Active-LOW momentary push to `GND` ($50\text{ ms}$ debounce) |
| **Ped Button 2** | Ped 2 Pushbutton | `PIN_PED2_BUTTON` | **GPIO 4** | **Right Pin 11** (`D4`)| `INPUT_PULLUP` | Active-LOW momentary push to `GND` ($50\text{ ms}$ debounce) |
| **IR Sensor 1** | Main Road Sensor 1 | `PIN_MAIN_IR_SENSOR_1` | **GPIO 33** | **Left Pin 7** (`D33`) | `INPUT_PULLUP` | Active-LOW, internal pull-up supported |
| **IR Sensor 2** | Main Road Sensor 2 | `PIN_MAIN_IR_SENSOR_2` | **GPIO 34** | **Left Pin 4** (`D34`) | `INPUT` | Input-Only: **Requires external $10\text{ k}\Omega$ pull-up to 3.3V** |
| **IR Sensor 3** | Side Road Sensor 1 | `PIN_SIDE_IR_SENSOR_1` | **GPIO 35** | **Left Pin 5** (`D35`) | `INPUT` | Input-Only: **Requires external $10\text{ k}\Omega$ pull-up to 3.3V** |
| **IR Sensor 4** | Side Road Sensor 2 | `PIN_SIDE_IR_SENSOR_2` | **GPIO 36** | **Left Pin 2** (`VP`) | `INPUT` | Input-Only (`VP`): **Requires external $10\text{ k}\Omega$ pull-up to 3.3V** |
| **IR Sensor 5** | Turn Road Sensor 1 | `PIN_TURN_IR_SENSOR_1` | **GPIO 39** | **Left Pin 3** (`VN`) | `INPUT` | Input-Only (`VN`): **Requires external $10\text{ k}\Omega$ pull-up to 3.3V** |
| **IR Sensor 6** | Turn Road Sensor 2 | `PIN_TURN_IR_SENSOR_2` | **GPIO 15** | **Right Pin 13** (`D15`)| `INPUT_PULLUP` | `MTDO` strapping pin; idle Active-LOW sensor holds HIGH |

---

## 5. Electrical Driver Architecture

```
                                    +5V Power Rail
                                          |
                                    [Pin 9: VCC]
                              +-----------------------+
      ESP32 30-PIN MCU        |  TBD62783APG DRIVER   |         5V COMMON-GROUND
  ========================    |  (High-Side Sourcing) |        VEHICLE RYG MODULES
  GPIO 27 (Left Pin 10) ----->| IN1              OUT1 |-----> Main Red Anode
  GPIO 26 (Left Pin 9)  ----->| IN2              OUT2 |-----> Main Yellow Anode
  GPIO 25 (Left Pin 8)  ----->| IN3              OUT3 |-----> Main Green Anode
  GPIO 14 (Left Pin 11) ----->| IN4              OUT4 |-----> Side Red Anode
  GPIO 13 (Left Pin 13) ----->| IN5              OUT5 |-----> Side Yellow Anode
  GPIO 12 (Left Pin 12) ----->| IN6              OUT6 |-----> Side Green Anode
  GPIO 23 (Right Pin 1) ----->| IN7              OUT7 |-----> Turn Red Anode
  GPIO 22 (Right Pin 2) ----->| IN8              OUT8 |-----> Turn Yellow Anode
                              |      [Pin 10: GND]    |
                              +-----------------------+
                                          |
                                          +-----------------> Module Common GND
                                          |
  GPIO 21 (Right Pin 5) ----->[ Ch 9 Source / Transistor ]----> Turn Green Anode
                                          |
  ========================                |
  COMMON GROUND RAIL (GND) ---------------+-----------------> System Power GND
```

1. **Vehicle Groups (9 Lamps):**
   - 5V common-ground (cathode) assemblies with series resistors sized for $V_{CC} = 5.0\text{ V}$.
   - Red and Yellow illuminate directly from 3.3V due to low $V_f \approx 1.9\text{ V}$.
   - Green LEDs have high forward voltage drop ($V_f \approx 3.2\text{--}3.4\text{ V}$), drawing $< 0.6\text{ mA}$ at $3.3\text{ V}$ drive.
   - Sourcing $+5.0\text{ V}$ through a **Toshiba TBD62783APG** high-side driver yields $I_f \approx 12\text{ mA}$, achieving full green illumination.
2. **Pedestrian Groups (4 Lamps):**
   - Discrete LEDs driven via **ULN2003A** low-side Darlington sink with common $+5\text{ V}$ anode rail.
