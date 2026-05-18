# ESP32 Angle-Mode Quadcopter Flight Controller

![Photo 1](photo.jpg)
![Photo 2](photo2.jpg)

> 🎬 **Flight video:** [Watch on Google Drive](https://drive.google.com/file/d/1vH2mI2h-nlh5-8pSedQAtvSlQ2322_l2/view?usp=drive_link)

---

A custom angle-mode flight controller for **ESP32 (WROOM-32)**, written in Arduino C++. Features a full cascade PID stabiliser, altitude hold using LiDAR + barometer, position hold using optical flow, DShot300 ESC protocol, CRSF RC receiver, and live BLE telemetry.

---

## Hardware

| Component | Part | Interface |
|---|---|---|
| MCU | ESP32 WROOM-32 | — |
| IMU | MPU6050 | I2C `0x68` — ±500 °/s gyro, ±8 g accel |
| Barometer | BMP280 | I2C `0x76` |
| Optical Flow + LiDAR | MTF-02P | MAVLink @ 115200 baud, UART0 RX=GPIO3 |
| RC Receiver | ExpressLRS / CRSF | UART2 RX=GPIO16, TX=GPIO17 @ 921600 baud |
| ESCs | 4× DShot300 | GPIO 25, 26, 32, 33 |
| Status LED | — | GPIO 15 |

### Motor Layout

```
        FRONT
   FL(26)  FR(33)
    CW      CCW
   BL(25)  BR(32)
    CCW     CW
        REAR
```

| Motor | GPIO | Position | Spin |
|---|---|---|---|
| M1 `escFR` | GPIO 33 | Front-Right | CCW |
| M2 `escBR` | GPIO 32 | Rear-Right | CW |
| M3 `escBL` | GPIO 25 | Rear-Left | CCW |
| M4 `escFL` | GPIO 26 | Front-Left | CW |

> Verify wiring against `ESC_PWM_Test/ESC_PWM_Test.ino` before first flight.

### I²C Bus

| Signal | GPIO |
|---|---|
| SDA | GPIO 21 (default) |
| SCL | GPIO 22 (default) |

BMP280 initialised at 100 kHz; bus returned to 400 kHz for MPU6050.

---

## Transmitter — BetaFPV LiteRadio 3 / 3 Pro

```
         LiteRadio 3 Pro (Mode 2)

   SWA ─┐               ┌─ SWB
  (CH5) │               │ (CH6)
        │               │
  ┌─────┴───────────────┴─────┐
  │                           │
  │  LEFT STICK  RIGHT STICK  │
  │  Thr  Yaw    Roll  Pitch  │
  │  (↕)  (↔)    (↔)   (↕)   │
  │                           │
  │   SWC ──────────── SWD    │
  │  (CH7)           (CH8)    │
  └───────────────────────────┘
```

### RC Channel Mapping

| Channel | Switch / Stick | Function |
|---|---|---|
| CH1 | Right stick ↔ | Roll |
| CH2 | Right stick ↕ | Pitch |
| CH3 | Left stick ↕ | Throttle |
| CH4 | Left stick ↔ | Yaw |
| CH5 | **SWA** (top-left) | Arm / Disarm |
| CH6 | **SWB** (top-right) | Altitude Hold on/off |
| CH8 | **SWD** (bottom-right) | Altitude Hold (secondary) |

---

## Features

### Angle Mode
Roll and pitch use a **complementary filter** (τ = 6.0 s) fusing gyro integration with accelerometer-derived angle. Clamped to ±20°.

### Cascade PID
All three axes use a **two-loop cascade**:
- **Outer loop** — angle error → desired rate setpoint
- **Inner loop** — rate error → motor mix output

### Yaw Heading Hold
Yaw stick rotates the heading target at stick rate. Releasing the stick holds the last heading.

### Altitude Hold
- Activated by CH6 (or CH8) > 1500 µs while armed and throttle > 1200 µs
- **Sensor priority:** MTF-02P LiDAR primary, BMP280 fallback
- **2-loop cascade:** outer altitude P → desired climb rate; mid rate PI → throttle correction
- PID gates on each fresh sensor sample — no integrator windup at 250 Hz loop rate
- Stick nudge (> 35 µs from base throttle) shifts the altitude target in real time

### Position Hold
Activates automatically when altitude hold is `ACTIVE` and optical flow quality > 40:
- Velocity PI using MTF-02P optical flow
- Extra-heavy smoothing (α = 0.05) to prevent oscillations
- Pilot stick input (> 50 µs deflection) temporarily disengages position hold

### Live IMU Calibration
On every boot: 2000 IMU samples → gyro bias offsets, accelerometer offsets, and physical mount tilt are computed automatically. No manual calibration needed.

---

## PID Gains

### Angle (outer loop)

| Axis | P | I | D |
|---|---|---|---|
| Roll / Pitch | 2.0 | 0.2 | 0.05 |
| Yaw | 2.0 | 0.1 | 0.0 |

### Rate (inner loop)

| Axis | P | I | D |
|---|---|---|---|
| Roll / Pitch | 1.1 | 1.5 | 0.0005 |
| Yaw | 4.0 | 0.3 | 0.005 |

### Altitude Hold

| Loop | Gain | Value |
|---|---|---|
| Outer altitude P | P_ALT | 0.6 |
| Mid rate PI | P_RATE_THR | 1.0 |
| Mid rate PI | I_RATE_THR | 0.25 |
| Correction clamp | — | ±220 PWM |

---

## Build Flags

Defined near the top of the sketch:

| Define | Default | Purpose |
|---|---|---|
| `ALT_HOLD_ENABLE` | `1` | Enable/disable altitude hold entirely |
| `MOTOR_TEST_ENABLE` | `0` | Spin each motor in sequence for orientation check |
| `MANUAL_THROTTLE_MODE` | `0` | Throttle via Serial instead of CRSF (bench testing) |

> ⚠ **REMOVE PROPELLERS** before setting `MOTOR_TEST_ENABLE 1` or `MANUAL_THROTTLE_MODE 1`.

---

## Arm / Disarm

| Action | Switch | Condition |
|---|---|---|
| **Arm** | **SWA** → UP | SWA in up position **AND** throttle (left stick) at bottom |
| **Disarm** | **SWA** → DOWN | SWA in down position **AND** throttle below mid |

On disarm, all PID integrators and angle states are reset; motors receive DShot 0.

---

## Throttle Constants

| Constant | Value | Purpose |
|---|---|---|
| `ThrottleIdle` | 1140 | Minimum motor speed while armed |
| `ThrottleLanding` | 1100 | Gentle landing floor |
| `ThrottleCutOff` | 1000 | Disarmed — DShot 0 sent |

---

## BLE Telemetry

Connect with any BLE UART terminal app (e.g. **Serial Bluetooth Terminal** on Android) to the device named **`OpticalFlowADE`**.

**Nordic UART Service:**
- Service UUID: `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`
- TX Characteristic: `6E400003-B5A3-F393-E0A9-E50E24DCCA9E`

Data streams at ~40 Hz:
```
Vel X: -0.12 cm/s | Vel Y: 0.05 cm/s | Vel Z: 1.23 cm/s | Quality: 210 |
Dist: 42.50 cm | Rel Alt: 12.30 cm | Fused H: 42.50 cm (LiDAR) |
Temp: 27.45 C | Press: 101325.00 Pa | Alt: 142.30 m | STATE: ALT_HOLD_ACTIVE | ARM: 1
```

---

## Python GUI Monitor

A desktop BLE monitor is included at `optical_flow_bmp280_monitor.py`.

- Connects over BLE to `OpticalFlowADE` automatically
- Displays live: Vel X/Y/Z, flow quality, distance, relative altitude, fused height, temperature, pressure, flight state
- **Live Plots** tab: fused height, flow quality, Vel Z

```bash
pip install -r requirements-monitor.txt
python optical_flow_bmp280_monitor.py
```

---

## Required Libraries

Install via Arduino Library Manager:

| Library |
|---|
| `Adafruit BMP280 Library` |
| `Adafruit Unified Sensor` |
| `DShotRMT` |
| `MAVLink` (mavlink/c_library_v2) |
| ESP32 BLE Arduino (bundled with ESP32 core) |

**Board:** ESP32 Dev Module — Arduino ESP32 core **3.3.6** or newer.

---

## Build & Flash

1. Open `Anglemode_flightcontrollerModified.ino` in Arduino IDE
2. **Board:** ESP32 Dev Module
3. **Upload Speed:** 921600
4. Select the correct COM port and click **Upload**

> The file must be saved as **UTF-8 without BOM**. The `.vscode/settings.json` in this repo enforces this for VS Code. A BOM causes `error: '﻿' does not name a type` on line 1 during compilation.

---

## Loop Rate

Main loop runs at **250 Hz** (4 ms period), enforced with a busy-wait. Actual `dt` is measured each iteration and clamped to 50 ms max to guard against startup transients.

---

## Operational Manual

### 1. Pre-Flight Checklist

- [ ] Props securely tightened and in correct orientation (CW/CCW per motor layout)
- [ ] Battery fully charged and XT60 connector inspected
- [ ] RC transmitter powered on, ELRS link established (solid LED on receiver)
- [ ] CH5 arm switch in the **down / disarmed** position (< 1500 µs)
- [ ] Throttle stick at **minimum**
- [ ] Drone placed on a **flat, level surface** — the IMU calibrates on boot
- [ ] Clear area of at least 3 m in all directions

---

### 2. Power-On / Boot Sequence

Connect the battery. The status LED on **GPIO 15** will flash to indicate boot progress:

| Phase | LED | Duration | What's happening |
|---|---|---|---|
| BLE init | — | ~1 s | BLE advertising starts |
| IMU calibration | — | ~2 s | 2000 gyro + accel samples taken — **keep the drone perfectly still** |
| ESC arming | — | ~1.2 s | DShot 0 sent to all ESCs |
| ESC wait | — | ~5 s | Continuous DShot 0 — ESCs complete their startup beep sequence |
| Ready | **ON** | — | Single long blink — flight controller is ready |

> ⚠ **Do not move or tilt the drone during the IMU calibration phase.** Any movement during the first ~8 seconds will cause gyro bias and level-trim errors, resulting in a drifting or oscillating drone.

---

### 3. Arming

1. Push the **left stick (throttle) to the bottom**
2. Flip **SWA** (top-left switch) **UP**
3. Motors begin spinning at idle speed — you will hear them start

> If the drone does not arm: confirm SWA is fully up, throttle stick is at the very bottom, and the boot sequence has completed (you heard the ESC beeps).

To **disarm**: flip **SWA DOWN** while the **throttle stick is below mid**. All motors stop immediately and all PID state is reset.

---

### 4. Normal Flight (Stabilise / Angle Mode)

| Stick | Direction | Effect |
|---|---|---|
| Right stick ↔ | Left / Right | Roll left / right |
| Right stick ↕ | Forward / Back | Pitch forward / back |
| Left stick ↔ | Left / Right | Yaw (rotate) left / right |
| Left stick ↕ | Up / Down | Increase / decrease throttle |

- The drone **self-levels** on roll and pitch — release the sticks and it returns to flat
- Maximum lean angle is **±20°**
- Yaw stick rotates the heading target at stick rate; releasing the stick **holds the current heading**
- Throttle is direct — you control altitude manually

---

### 5. Altitude Hold

**To activate:**
1. Fly to your desired hold altitude
2. Centre the **left stick (throttle)** roughly at mid position
3. Flip **SWB** (top-right switch) **UP**
4. BLE state shows `ALT_HOLD_READY` → then `ALT_HOLD_ACTIVE` once throttle is above low-stick

**While active:**

| Left stick (throttle) position | Effect |
|---|---|
| Centred (small deflection < 35 µs from base) | Altitude held — no movement |
| Pushed up (large deflection) | Altitude target **rises** |
| Pulled down (large deflection) | Altitude target **descends** |

- Sensor shown on BLE telemetry: `LiDAR` (MTF-02P active) or `BMP` (barometer fallback)

**To deactivate:** Flip **SWB DOWN**. Throttle immediately returns to direct stick control.

> ⚠ Do not activate altitude hold below ~30 cm — the LiDAR needs a valid reading and the integrator needs time to settle.

---

### 6. Position Hold

Position hold activates **automatically** — no switch needed — when all of the following are true:

- Drone is **armed**
- Altitude hold is **ACTIVE**
- MTF-02P optical flow quality > 40 (shown as `Quality` in BLE telemetry)

**In position hold:**
- The drone corrects horizontal drift using optical flow velocity
- Moving the roll/pitch stick more than 50 µs from centre **overrides** position hold and lets you fly freely; it re-engages automatically when you re-centre the sticks
- For best results fly at 0.3–2 m above a textured surface in good lighting

**If flow quality drops** (e.g. flying over a featureless surface or in poor light), position hold disengages gracefully and the drone reverts to pure angle mode.

---

### 7. Landing

#### Manual landing
1. Reduce throttle (left stick down) slowly to descend
2. On touchdown, push left stick to the **bottom**
3. Flip **SWA DOWN** to disarm — motors stop

#### Landing with altitude hold active
1. Hold left stick slightly **below centre** to nudge the altitude target downward
2. When ~20–30 cm from the ground, flip **SWB DOWN** to exit altitude hold
3. Descend manually, push left stick to bottom, flip **SWA DOWN** to disarm

> ⚠ Never rely on altitude hold to touch down — always disengage SWB and land manually for the final 20–30 cm.

---

### 8. BLE Telemetry Monitoring (Optional)

1. Open **Serial Bluetooth Terminal** (Android) or any BLE UART app
2. Scan and connect to **`OpticalFlowADE`**
3. Data streams automatically at ~40 Hz

Useful values to watch during flight:

| Field | What to look for |
|---|---|
| `STATE` | `STABILIZE` → `ALT_HOLD_ACTIVE` on CH6 flip |
| `Quality` | > 40 for position hold; > 100 is excellent |
| `Rel Alt` | Relative altitude in cm from home point |
| `Vel Z` | Vertical speed in cm/s — should settle near 0 in altitude hold |
| `ARM` | `1` = armed, `0` = disarmed |

---

### 9. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Won't arm | CH5 not high, or throttle not at min | Check TX switch mapping and throttle position |
| Drifts on one axis after arming | Drone moved during IMU calibration | Power cycle, keep drone still during boot |
| Oscillates at hover | PID gains too high | Reduce `PRateRoll`/`PRatePitch` D term first |
| Altitude hold overshoots | `P_ALT` or `P_RATE_THR` too high | Lower `P_ALT` from 0.6 toward 0.4 |
| Position hold causes oscillation | Flow noise too high | Fly higher (> 40 cm) or reduce `P_PH` |
| `'﻿' does not name a type` compile error | BOM in .ino file | See Build & Flash note; re-strip BOM |
| Motors don't spin after arming | ESC arming timed out | Power cycle; ensure DShot 0 is sent during boot wait |

---

## Disclaimer

> THE SOFTWARE IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND. THE USER ASSUMES ALL RISK. Flying multirotors carries risk of injury and property damage. Always fly responsibly and within local regulations.

