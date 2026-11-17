# TPMS Diagnostic Tool

> **Copyright (c) 2026 Michael Schader. All Rights Reserved.** See [LICENSE](./LICENSE).

Standalone, pocket-sized dual-band (315 MHz and 433.92 MHz) Tire Pressure Monitoring System (TPMS) diagnostic receiver, packet decoder, and close-range 125 kHz LF sensor wake tool built strictly from existing bench inventory.

---

## Project Seed Document

### 1. Project Purpose & System Overview

#### Goal
Build a standalone, pocket-sized dual-band (315 MHz and 433.92 MHz) Tire Pressure Monitoring System (TPMS) diagnostic receiver, packet decoder, and close-range 125 kHz LF sensor wake tool built strictly from existing bench inventory. The platform requires no external display, no Flipper Zero, and no discrete capacitors or diodes.

#### Functional Capabilities
- **Dual-Band UHF Reception:** Frequency hopping between 315.00 MHz (North American domestic/Asian OEMs) and 433.92 MHz (European, late-model domestic, and programmable universal sensors).
- **Raw Bitstream Demodulation & Decoding:** Asynchronous capture via the CC1101 GDO0 pin to decode multi-vendor FSK and ASK/OOK Manchester-encoded TPMS frames. Telemetry payload extraction includes Sensor ID, tire pressure in PSI/kPa, internal tire temperature, battery status, fast-leak/motion flags, and CRC-8 validation.
- **Zero-Display UI & Telemetry Streaming:** Live streaming of JSON telemetry over Bluetooth Low Energy (BLE Nordic UART Service / WebBLE) and USB Serial to a smartphone or laptop dashboard, complemented by onboard LED packet delivery confirmation.
- **On-Demand 125 kHz LF Sensor Wake:** Integrated close-range 125 kHz LF inductive wake stage utilizing a capacitor-free resistor-damped RL loop. Triggers stationary sleeping sensors at the tire sidewall, offering an alternative to pressure-drop ($\Delta P$) and wheel-rotation wake mechanisms.

---

### 2. Inventory Hardware Bill of Materials (BOM)

| Component | Exact Part / Spec | Role & Electrical Notes |
| --- | --- | --- |
| **Microcontroller & BLE** | Adafruit Bluefruit nRF52 Feather | nRF52832 SoC, 3.3V logic, native BLE 5.0, onboard 200 mA LiPo charger ([Adafruit Bluefruit nRF52 Feather Pinout](https://learn.adafruit.com/bluefruit-nrf52-feather-learning-guide/device-pinout)). |
| **Sub-GHz RF Transceiver** | Generic 2x4 CC1101 Module | Variant A / E07-M1101D pin layout. Pin 1 (square pad) = GND, Pin 2 = VCC. ~300 k$\Omega$ quiescent resistance between Pin 1 and Pin 2. |
| **UHF Antenna** | 2 dBi 433 MHz SMA Male Rubber Duck | Resonant on 433.92 MHz; provides sufficient handheld wheel-well sensitivity at 315 MHz. |
| **Battery** | 501522 1S 3.7V 200 mAh LiPo Pouch | 5.0 × 15 × 22 mm with PCM. ~25 mA continuous RX+BLE draw (~7.5–8 hrs runtime); ~65 mA peak with 125 kHz wake burst. Charges at 200 mA (1.0C). |
| **125 kHz LF Wake Coil** | Hand-Wound Air-Core Coil | 30 AWG magnet wire. 35 turns on 33.4 mm form or 42 turns on 25.0 mm form (~63 $\mu$H, ~1.1–1.4 $\Omega$ DCR, ~50 $\Omega$ reactance at 125 kHz). |
| **LF Low-Side MOSFET** | FQP30N06L N-Channel MOSFET | TO-220 package, logic-level $V_{DS} = 60\text{V}$, $I_D = 32\text{A}$, $V_{GS(\text{th})} = 1.0\text{--}2.5\text{V}$. |
| **Resistors (4 total)** | 3× 100 $\Omega$, 1× 10 k$\Omega$ | 2× 100 $\Omega$ parallel (50 $\Omega$) for VBAT feed; 1× 100 $\Omega$ coil flyback clamp ($V_{DS} < 12\text{V}$); 1× 100 $\Omega$ series Gate; 1× 10 k$\Omega$ Gate pull-down to GND. |

*(Note: 3× 100 $\Omega$ covers the 2× 100 $\Omega$ parallel feed + 1× 100 $\Omega$ coil clamp or series Gate per bench inventory notes; refer to the wiring map below for the complete resistor placement.)*

---

### 3. Master Pin Assignment & Wiring Map

#### CC1101 2x4 Header to Bluefruit nRF52 Feather
Orientation: Component side facing operator, SMA antenna pointing right, square solder pad (Pin 1) at Top-Left.

| CC1101 Pin | Header Position | Function | Feather Pin Connection |
| --- | --- | --- | --- |
| **Pin 1** | Top-Left (Square Pad) | GND | Feather `GND` (0 $\Omega$ continuity to SMA outer barrel) |
| **Pin 2** | Top-Right | VCC | Feather `3V` (Strictly 3.3V regulated rail) |
| **Pin 3** | 2nd Down, Left | GDO0 | Feather `15` (`P0.15` — Async raw demodulated pulse interrupt) |
| **Pin 4** | 2nd Down, Right | CSN | Feather `30` (`P0.30` — Custom remapped SPI chip select) |
| **Pin 5** | 3rd Down, Left | SCK | Feather `11` (`P0.11` — Custom remapped SPI clock) |
| **Pin 6** | 3rd Down, Right | MOSI (SI) | Feather `MOSI` (`P0.13` — Hardware SPI data out) |
| **Pin 7** | Bottom-Left | MISO (SO) | Feather `MISO` (`P0.14` — Hardware SPI data in) |
| **Pin 8** | Bottom-Right | GDO2 | Unconnected (`RADIOLIB_NC`) |

#### 125 kHz LF Wake Driver & Power Wiring (Capacitor-Free RL Stage)
- **Power Source:** 501522 200 mAh LiPo connected to Feather JST-PH 2.0 mm socket.
  - Orientation (socket facing user, USB port pointing away): Left Pin = Positive (Red, `+`), Right Pin = Ground (Black, `-`).
- **High-Side Coil Feed:** Feather `BAT` pin $\rightarrow$ 2× 100 $\Omega$ resistors in parallel (50 $\Omega$ total) $\rightarrow$ High-side lead of 30 AWG coil + top leg of parallel 100 $\Omega$ clamp resistor.
- **Low-Side Coil / Drain Junction:** Low-side lead of 30 AWG coil + bottom leg of parallel 100 $\Omega$ clamp resistor $\rightarrow$ FQP30N06L Pin 2 (Drain / Metal Tab).
- **Gate Drive (FQP30N06L Pin 1):** Feather Pin `27` (`P0.27`) $\rightarrow$ 100 $\Omega$ series resistor $\rightarrow$ Pin 1 (Gate). A 10 k$\Omega$ pull-down resistor is wired between Gate and `GND`.
- **Source (FQP30N06L Pin 3):** Wired directly to Feather `GND`.

#### Reserved & Unassigned Feather Pins
- **Reserved (Do Not Connect):**
  - Pin `7`: Factory Reset (`FRST`) monitoring during boot.
  - Pin `31` (`P0.31` / `A7`): Hardwired internal 100k/100k battery voltage divider rail.
- **Manual LF Wake Button (`PIN_WAKE_BUTTON` = `20` / `DFU` / `P0.20`):** Sampled by the bootloader only at reset/power-on (must remain open/`HIGH` at boot); at runtime a momentary active-`LOW` press triggers a `PULSED` 125 kHz LF wake burst (`5× 15 ms pulses + 100 ms CW tail`, 35 ms debounce).
- **Unassigned / Available:** Feather `SCK` (`P0.12`), Feather Pin `16` (`P0.16`), Analog Pins `A0`–`A5`, $\text{I}^2\text{C}$ lines (`SDA`, `SCL`).

---

### 4. Mechanical Layout & Hardware Safety Guidelines

- **MOSFET Isolation:** Insulate the FQP30N06L TO-220 metal tab using heat-shrink tubing or Kapton tape. The tab is electrically tied to the Drain (live VBAT when switching is idle). Bend the leads 90 degrees so the assembly lies flush against the enclosure frame.
- **RF Isolation:** Maintain a physical separation of at least 6 cm to 8 cm between the front 30 AWG LF wake coil and the rear CC1101 transceiver module / 2 dBi rubber duck antenna. This minimizes near-field antenna detuning and high-current LF switching interference.
- **Thermal Management:** Position the 501522 LiPo cell flat beneath the center of the Feather mainboard. Maintain a small air gap to preserve thermal safety margins during 1.0C (200 mA) USB fast charging.
- **Strain Relief:** Securely anchor the CC1101 edge-mounted SMA connector barrel. Apply strain relief to the stripped 30 AWG magnet wire terminations prior to closing the enclosure chassis.

---

### 5. Firmware Architecture & Pin Definitions

- **Software Stack:** Built on Arduino / PlatformIO targeting the Adafruit nRF52 BSP (`adafruit_feather_nrf52832`). Sub-GHz radio control uses `RadioLib` (`jgromes/RadioLib`) with `SPI.setPins(...)` remapped hardware SPI (`SCK = 11`, `MISO = MISO`, `MOSI = MOSI`).
- **RF & Decoder Pipeline:**
  - Radio configured via `beginFSK()` / `beginOOK()` + `receiveDirect()` asynchronous reception (2-FSK 35 kHz deviation, 135 kHz RX bandwidth, 19.2 kbps bitrate; ASK/OOK 9.6 kbps, 135 kHz RX bandwidth).
  - Software hopper alternates between `315.00 MHz` and `433.92 MHz` every ~400 ms (deferring briefly if an active pulse burst is underway) and optionally cycles FSK/OOK modulations.
  - `PIN_CC1101_GDO0` (`CHANGE` interrupt) timestamps pulse widths at 1 µs resolution using `NRF_TIMER1` (free-running 32-bit 1 MHz timer, unaffected by FreeRTOS idle `__WFE()`) into a 2048-entry lock-free SPSC ring buffer.
  - Manchester decoder extracts frames, validates CRC-8 / checksums, and parses Schrader, Toyota/Pacific, Ford, or generic Manchester TPMS payloads.
- **LF Wake & Telemetry Interfaces:**
  - `HwPWM2` drives 125.000 kHz (16 MHz / 128, 50% duty = 64) on Feather Pin `27` (`P0.27`), with strict idle `LOW` clamp and cooldown enforcement.
  - Single-line JSON telemetry streams simultaneously over BLE Nordic UART Service (`TPMS-Diag`) and USB Serial (`115200` baud), with `LED_RED` blinking on decoded packets and `PIN_VBAT_SENSE` (`P0.31`) reporting battery status via BLE Battery Service and JSON status packets.

```cpp
#define PIN_CC1101_SCK   11   // P0.11 (custom remapped SPI clock)
#define PIN_CC1101_MOSI  MOSI // P0.13 (default hardware MOSI)
#define PIN_CC1101_MISO  MISO // P0.14 (default hardware MISO)
#define PIN_CC1101_CSN   30   // P0.30 (custom remapped chip select)
#define PIN_CC1101_GDO0  15   // P0.15 (async pulse capture interrupt)
#define PIN_CC1101_GDO2  RADIOLIB_NC
#define PIN_LF_WAKE_PWM  27   // P0.27 -> 100R -> FQP30N06L Gate drive
#define PIN_VBAT_SENSE   31   // P0.31 (onboard 100k/100k voltage divider)
```

---

### 6. Pre-Power & First-Power Verification Checklist

1. **Continuity Validation:** Verify 0 $\Omega$ DC resistance across all 7 physical CC1101 interconnect lines to the Feather board. Confirm open-circuit conditions (OL) between all adjacent pins.
2. **Unpowered Resistance Checks:**
   - Feather `3V` to `GND`: Target ~300 k$\Omega$.
   - Feather Pin `27` to `GND`: Target ~10.1 k$\Omega$ (verifies series Gate path and pull-down resistor integrity).
   - Feather `BAT` to FQP30N06L Drain: Target ~51.2 $\Omega$ (50 $\Omega$ parallel network + coil DCR).
   - FQP30N06L Metal Tab to `GND` / `3V` / SMA Shell: Target Open Circuit (OL).
3. **First-Power Procedure:**
   - Thread the 2 dBi 433 MHz rubber duck antenna onto the SMA barrel before powering on.
   - Connect USB cable; measure logic rail voltage to ensure exactly 3.30V across `3V` and `GND`.
   - Plug in the 501522 LiPo cell (Confirm orientation: Red = Left/Positive, Black = Right/Ground).
   - Confirm that the onboard amber charge LED (`CHG`) illuminates steadily during active charging.

---

## Build, Flash & Serial/BLE Command Reference

### Building & Flashing with PlatformIO

```bash
# Build firmware
pio run

# Upload over USB (Adafruit nRF52 Feather bootloader)
pio run -t upload

# Open USB serial monitor (115200 baud)
pio device monitor -b 115200
```

### Project Layout

```text
TPMS tool/
├── LICENSE                 # Proprietary copyright notice (All Rights Reserved)
├── .gitignore              # Ignores .pio/, .vscode/, temp/backup files
├── index.html              # Field-ready WebBLE (NUS + BAS) & Web Serial PWA dashboard
├── manifest.json           # PWA manifest for Android Chrome / Desktop home-screen install
├── sw.js                   # Network-first Service Worker for offline field cache
├── icon.svg                # PWA application icon
├── platformio.ini          # PlatformIO config (adafruit_feather_nrf52832 + RadioLib)
├── README.md               # Seed documentation, wiring map, command & JSON reference
├── dashboard/
│   └── index.html          # Legacy path redirect to ../index.html
├── include/
│   ├── config.h            # Hardware pins, RF defaults, LF PWM & battery thresholds
│   ├── power_monitor.h     # SAADC battery voltage & SoC monitor (Pin 31 / A7)
│   ├── lf_wake.h           # 125 kHz LF wake burst driver (HwPWM2 on Pin 27)
│   ├── rf_receiver.h       # CC1101 RadioLib driver, band/mod hopper, 1 us TIMER1 pulse capture
│   ├── tpms_decoder.h      # Manchester decoder & Schrader/Toyota/Ford/Continental/Raw parsers
│   └── telemetry_ble.h     # BLE NUS + DIS + BAS + USB Serial JSON interface & CLI parser
└── src/
    ├── main.cpp
    ├── power_monitor.cpp
    ├── lf_wake.cpp
    ├── rf_receiver.cpp
    ├── tpms_decoder.cpp
    └── telemetry_ble.cpp
```

### WebBLE / Web Serial Dashboard (`index.html`)

- **Live GitHub Pages (Field Testing — Android Chrome / Desktop Chrome / Edge):** Open `https://thefoeyouknow.github.io/tpms-diagnostic-tool/` over HTTPS and click **Connect BLE (TPMS-Diag)**. Because it is served over HTTPS with `manifest.json` and `sw.js`, you can use **Add to Home screen** / **Install app** in Chrome for full-screen standalone operation, Wake Lock, audio/vibration alerts, and saved **Vehicle + Trailer Garage Profiles** (`FL`/`FR`/`RL`/`RR`/`SPARE`/`AUX` for vehicles and `L1`/`R1`/`L2`/`R2`/`L3`/`R3` for trailers, soft limit of 6 profiles and 6 saved sensors per profile, simultaneous Tow Vehicle + Trailer monitoring, single-slot tap-to-learn + Auto-Learn Walk-Around, and persistent pinned cards protected from eviction by passing transient traffic). *(Note: Web Bluetooth is supported on Android Chrome, Windows/macOS/Linux Chrome/Edge, and Bluefy on iOS; Safari on iOS does not expose Web Bluetooth.)*
- **Local HTTPS-equivalent Origin (`http://localhost`):** Serve locally with `python -m http.server 8000` in the project directory and open `http://localhost:8000/` (or `http://localhost:8000/dashboard/`, which redirects to `../index.html`).
- **USB Serial (`115200` baud):** On desktop Chrome or Edge, click **Connect USB Serial** and select the Feather COM port (works from `https://`, `http://localhost`, or `file://`).

### Serial & BLE NUS Commands (Newline-Terminated, Case-Insensitive)

| Command | Description |
| --- | --- |
| `HELP` | Print supported commands as a JSON `help` packet. |
| `PING` | Emit immediate lightweight `pong` JSON packet (`freq_mhz`, `rssi_dbm`, `vbat_v`, `ts_ms`) to verify BLE/Serial link latency. |
| `STATUS` | Emit immediate `status` JSON packet (band, mod, RSSI, VBAT, LF state, counters). |
| `DIAG` | Run hardware bring-up diagnostics (CC1101 chip version, 17-pin matrix scan, 315/433 RSSI, VBAT, LF PWM, pulse rate). |
| `PINLOW` / `PINHIGH` | Bench check: drives `CSN (30)`, `SCK (11)`, and `MOSI (13)` `LOW` or `HIGH` and reads `MISO`/`GDO0`. |
| `WAKE` | Trigger default 125 kHz LF wake burst (`CW`, 150 ms). |
| `WAKE CW [ms]` | Trigger continuous 125 kHz LF wake burst for `[ms]` (default 150 ms, max 500 ms). |
| `WAKE PULSE` | Trigger pulsed 125 kHz LF wake preamble (5× 15 ms pulses / 10 ms gaps + 100 ms CW). |
| `WAKE AUTO ON [ms]` | Enable periodic automatic 125 kHz LF wake bursts every `[ms]` (default 5000 ms, min 1500 ms). |
| `WAKE AUTO OFF` | Disable periodic automatic 125 kHz LF wake bursts. |
| `BAND 315` | Lock receiver to `315.00 MHz`. |
| `BAND 433` | Lock receiver to `433.92 MHz`. |
| `BAND HOP` | Hop between `315.00 MHz` and `433.92 MHz` every ~400 ms (default). |
| `MOD FSK` | Lock CC1101 demodulator to `2-FSK` (19.2 kbps, 35 kHz dev, 135 kHz BW). |
| `MOD OOK` | Lock CC1101 demodulator to `ASK/OOK` (9.6 kbps, 135 kHz BW). |
| `MOD BOTH` | Alternate `FSK` and `OOK` across hop cycles (default). |
| `RAW ON` / `RAW OFF` | Enable or disable emitting `RAW_MANCHESTER` candidate packets for unrecognized frames. |
| `TESTPKT [SCHRADER\|TOYOTA\|FORD]` | Inject a synthetic Manchester-encoded TPMS pulse train (`Schrader` default, `Toyota`, or `Ford`) into the decoder for self-testing without an RF source. |

### JSON Telemetry Schemas

All messages are emitted as single-line NDJSON objects over both USB Serial (115200 baud) and BLE Nordic UART Service (`TPMS-Diag`):

#### Decoded TPMS Packet (`type: "tpms"`)
```json
{"type":"tpms","proto":"Schrader","id":"0x01A2B3C4","freq_mhz":315.00,"mod":"FSK","rssi_dbm":-68.5,"pressure_psi":32.5,"pressure_kpa":224.1,"temp_c":24.0,"bat_low":false,"motion":true,"fast_leak":false,"crc_ok":true,"bits":64,"raw":"...","ts_ms":12345}
```

#### System Status / Heartbeat (`type: "status"`)
```json
{"type":"status","freq_mhz":433.92,"band_mode":"HOP","mod":"FSK","mod_mode":"BOTH","rssi_dbm":-96.0,"radio_ok":true,"vbat_v":4.08,"vbat_mv":4082,"soc_pct":87,"low_bat":false,"ble_conn":true,"raw_mode":false,"lf_active":false,"lf_auto":false,"lf_bursts":0,"pulses":1420,"dropped":0,"pkts":3,"ts_ms":15000}
```

#### LF Wake Burst Confirmation (`type: "lf_wake"`)
```json
{"type":"lf_wake","pattern":"CW","duration_ms":150,"freq_hz":125000,"duty_pct":50,"auto":false,"vbat_v":4.07,"ts_ms":18240}
```

#### Hardware Bring-Up Diagnostics (`type: "diag"`)
```json
{"type":"diag","radio_ok":true,"cc1101_ver":"0x14","rssi_315_dbm":-98.5,"rssi_433_dbm":-95.0,"vbat_v":4.10,"vbat_mv":4100,"soc_pct":89,"lf_pwm_hz":125000,"lf_pin":27,"pulses":1420,"dropped":0,"ble_conn":true,"ts_ms":20100}
```

---

## 125 kHz LF Wake Stage — Rev B (`TC4420CPA` DIP-8 + `68 µH` / `22 nF` Series-Resonant LC)

Compact, low-part-count replacement for the discrete TO-220 `FQP30N06L` / hand-wound coil stage. Works out-of-the-box with the existing `125.000 kHz` PWM firmware (`PIN_LF_WAKE_PWM = 27`).

### Rev B Bill of Materials (4 Core Parts)

| Qty | Component | Spec / Example Part | Role |
| :---: | --- | --- | --- |
| **1** | **Gate / Coil Driver IC** | **Microchip `TC4420CPA`** (`DIP-8`) | 6 A non-inverting CMOS totem-pole driver (`3.3 V` compatible logic input, active high/low push-pull output). |
| **1** | **125 kHz Emitter Inductor** | **`68 µH` Unshielded Radial Ferrite Choke** (e.g., `MECCANIXITY 68µH 8×15 mm` or `Bourns 5800-680-RC`) | Open-core ferrite inductor ($X_L \approx 53.4\ \Omega$ at `125 kHz`); radiates dipole flux from the cylinder end. |
| **1–2** | **Resonant / Bypass Capacitor** | **`22 nF` (`223`) 50 V Ceramic** | Series resonant cap ($X_C \approx 57.9\ \Omega$ at `125 kHz`) cancels coil reactance and blocks DC (`0 mA` idle). Optional 2nd `22 nF` across `VDD`–`GND` for local decoupling. |
| **1–2** | **Series Damping Resistor** | **`50 Ω`** (`2× 100 Ω` parallel) or **`1× 100 Ω`** 1/4W | Limits peak current draw from the 1S LiPo during `125 kHz` bursts. |

### Top-Down `TC4420CPA` (DIP-8) Wiring Diagram

```text
                          TC4420CPA (DIP-8)
                            ┌───‿───┐
      Feather BAT ──────────┤ 1 • 8 ├── VDD (optional 22 nF to Pin 5 GND)
                            │       │
   Feather Pin 27 (PWM) ────┤ 2   7 ├── OUT ───► [ 50–100 Ω ]
                            │       │                 │
             (Leave Open) ──┤ 3   6 ├── OUT       [ 22 nF ]
                            │       │                 │
      Feather GND ──────────┤ 4   5 ├── GND ◄─── ( 68 µH Coil )
                            └───────┘
```

### Pin-by-Pin Wiring Table

| `TC4420CPA` Pin | Name | Connection |
| :---: | --- | --- |
| **1** | `VDD` | Feather `BAT` (internally tied to Pin 8) |
| **2** | `IN` | Feather Pin `27` (`PIN_LF_WAKE_PWM`) |
| **3** | `NC` | Leave unconnected |
| **4** | `GND` | Feather `GND` (USB shield / JST `-` ground anchor; internally tied to Pin 5) |
| **5** | `GND` | Return lead of `68 µH` inductor (plus optional `22 nF` decoupling cap to Pin 8) |
| **6** | `OUT` | Internally tied to Pin 7 (leave open or share output) |
| **7** | `OUT` | `50–100 Ω` series resistor $\rightarrow$ `22 nF` (`223`) capacitor $\rightarrow$ `68 µH` inductor |
| **8** | `VDD` | Internally tied to Pin 1 (optional `22 nF` decoupling cap to Pin 5) |

