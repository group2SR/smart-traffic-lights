# Hardware Commissioning and Electrical Safety Checklist

> **CRITICAL RULE:** Do not energize physical vehicle lamp modules or enable unverified sensor channels until the corresponding hardware checks below are completed and verified with a multimeter.

---

## 1. Current Commissioning Status Summary

| Subsystem | Current Status | Commissioning Gate in Firmware |
| :--- | :--- | :--- |
| **Core FSM, Modes & Safety Invariants** | **Implemented & Tested in Software** | Active (`31` Unity unit tests passing). |
| **3 × 5V Common-Ground Vehicle RYG Modules** | **Blocked Pending Hardware Verification** | `VEHICLE_HIGH_SIDE_DRIVER_VERIFIED = false` (`UNVERIFIED_SAFE_DISABLED`). |
| **2 × Pedestrian RED/GREEN Signal Groups** | **Configured, Not Physically Validated** | `PED_ULN2003A_DRIVER_VERIFIED = false`. |
| **6 × IR Vehicle Presence Sensors (`IR1..IR6`)** | **Configured, Not Physically Validated** | `sensorValidatedMask = 0x00` (`PHYSICAL_IR_VALIDATED = false`). |
| **Protected Turn Movement (`TURN_GREEN`)** | **Configured, Not Physically Validated** | `protectedTurnCommissioned = false` (Right-Hand Traffic / Keep-Right). |

---

## 2. Step-by-Step Electrical Verification Checklist

### Step A — Power Distribution & Ground Reference
- [ ] Verify common `GND` continuity between the ESP32 DevKit V1 `GND`, the 5 V buck converter (`LM2596`) output `GND`, `ULN2003A` pin 8 (`E` / `GND`), and sensor grounds.
- [ ] Verify buck converter output voltage is adjusted to **`5.00 V ± 0.10 V`** **before** connecting the ESP32 or LED modules.
- [ ] Verify ESP32 `3V3` rail measures **`3.30 V ± 0.10 V`**.

### Step B — Vehicle Signal Modules (5 V Common-Ground)
- [ ] **DO NOT wire the 4-wire vehicle RYG modules (`GND, R, Y, G`) directly to `ULN2003A` outputs.** The vehicle modules have a common `GND` wire and require `+5V` sourcing on `R`, `Y`, and `G`, whereas `ULN2003A` is a low-side current sink.
- [ ] Install and verify a `+5V` high-side sourcing driver stage (e.g., `TBD62783APG` / `UDN2981A` high-side driver IC, or PNP/P-MOSFET high-side switches per channel).
- [ ] Confirm that when the ESP32 pin is in its configured inactive state, `0.0 V` appears on the lamp input, and when active, `+5V` is sourced through appropriate current limiting.
- [ ] Only after verifying all 9 vehicle channels, update `SignalOutputDriver::setVehicleDriverConfig(..., true)`.

### Step C — Pedestrian Signal LEDs (`ULN2003A` Low-Side Sink)
- [ ] Connect pedestrian LED anodes to `+5V` (common-anode topology).
- [ ] Connect each pedestrian LED cathode through an individual current-limiting resistor ($330\text{ }\Omega$ to $470\text{ }\Omega$, $\ge 0.25\text{ W}$) to its assigned `ULN2003A` output channel (`GPIO 19, 18, 17, 16`).
- [ ] Verify that pedestrian `WALK` (Green) and `DONT_WALK` (Red) illuminate individually and never simultaneously.
- [ ] Enable pedestrian output driving via `SignalOutputDriver::setPedestrianDriverConfig(OutputDriverType::LOW_SIDE_ULN2003A_ACTIVE_HIGH, true)`.

### Step D — `GPIO 12` (`MTDI`) Boot-Strap Rewiring Check
- [ ] If moving Side Road Yellow off `GPIO 12` (`MTDI`), move the physical wire from `GPIO 12` to **`GPIO 5`** (`PIN_SIDE_YELLOW_SAFE_PROPOSED = 5`) and enable `setUseSafeProposedSideYellowPin(true)`.
- [ ] Verify the ESP32 boots cleanly across 5 consecutive power-on resets without `VDD_SDIO` flash errors.

### Step E — Six IR Vehicle Presence Sensors (`IR1..IR6`)
- [ ] **5 V Level Protection**: Measure each IR sensor `OUT` pin when idle (no obstacle) and active (obstacle detected) **before** connecting to the ESP32. The voltage at the ESP32 pin must **never exceed `3.3 V`** (power IR sensors from `3.3 V` or use a $10\text{ k}\Omega / 20\text{ k}\Omega$ divider / series Schottky clamp to `3.3 V`).
- [ ] **External Pull-Ups on Input-Only Pins**: Verify external $10\text{ k}\Omega$ pull-up resistors to `3.3 V` are installed on `GPIO 34` (`IR2`), `GPIO 35` (`IR3`), `GPIO 36` (`IR4`), and `GPIO 39` (`IR5`).
- [ ] Validate each sensor individually in software (`setPhysicalIrChannelValidated(idx, true)`) and confirm via `SENSORS` serial command that `Stable=1` when blocked and `Chatter=0`.

### Step F — Right-Hand Traffic (Keep-Right) Protected Turn Commissioning
- [ ] Confirm the physical T-junction lane markings and signal head placement follow **Right-Hand Traffic (Keep-Right)**.
- [ ] Confirm that the Protected Turn movement (`TURN_GREEN`) is physically separated from conflicting through/side movements and that `ConflictMatrixConfig` reflects all physical conflict paths.
- [ ] Only after physical confirmation, call `setProtectedTurnCommissioned(nowMs, true)`.

