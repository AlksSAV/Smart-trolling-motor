# Smart Trolling Motor - Architecture

## Overview
Electronic anchor for a boat: when hold mode is active, the bow-mounted
brushless motor (ESC) keeps the boat in a set position. The motor is steered by
a servo, +/-45 deg from the centerline.

## Hardware

| Component | Part | Interface | Pins |
|-----------|------|-----------|------|
| MCU | ESP32-S3-N16R8 (16 MB flash, 8 MB PSRAM) | — | — |
| IMU | BNO085 | I2C (400 kHz) | SDA 12, SCL 13 |
| GNSS | u-blox UBX-M10050 (M10, SPG 5.10) | UART1 (38400) | TX 17 -> GNSS RX, RX 18 <- GNSS TX |
| ESC | brushless, bidirectional PWM | LEDC 50 Hz | GPIO 15 |
| Steering servo | digital PWM, +/-45 deg | LEDC 50 Hz | GPIO 16 |
| RC receiver | HotRC DS650 (6 ch) | GPIO input | 4-7, 10-11 (not integrated) |

Pin assignment: `docs/pinmap.md` (single source). Equipment registry: `docs/equipment.md`.

## Verified facts (on real hardware)
- BNO085 I2C address is `0x4B` (non-standard, typical for Chinese modules).
- UBX-M10050 runs at `38400 baud` (not 115200, not 9600), firmware `ROM SPG 5.10 (7b202e)`.
- Module configuration (read back with `tools/gnss_config.py`, saved RAM+BBR+Flash):
  `measRate` 200 ms (5 Hz), `dynModel` SEA (5), NMEA output on UART1 only `GGA` + `RMC`.
- Correction: earlier docs claimed "Sea dynamic model active"; the module actually
  reported `dynModel 6 (AIR1)` until it was explicitly set to SEA on 2026-10-11.
- Without a fix the module lowers its own rate to ~2.9-3.75 Hz (measured indoors);
  the configured 5 Hz applies when a fix is present.
- LEDC API (ESP32 Core v3.x): `ledcAttach(pin, freq, resolution)` + `ledcWrite(pin, duty)`.
- PWM: 14-bit resolution (16384 steps), 50 Hz = 20 ms period.

## Software

All firmware is in `src/main.cpp` (self-contained).

### Operating modes
1. **MANUAL** — ESC and servo at neutral. (RC control is a future task.)
2. **HEADING_HOLD** — IMU holds the current yaw; servo active, ESC neutral.
3. **ANCHOR_HOLD** — GNSS holds the position; both ESC and servo active.

Mode is switched by serial command: `M` (manual), `H` (heading hold),
`A` (anchor hold, requires a GNSS fix), `S` (anchor hold with a simulated target
for indoor testing).

### Control parameters (`src/main.cpp`)
| Parameter | Value |
|-----------|-------|
| Loiter radius (dead zone) | 2.0 m |
| Max distance (full speed) | 10.0 m |
| Servo P gain | 8.0 us/deg |
| ESC neutral / max forward | 1500 / 1800 us |
| Servo min / neutral / max | 1000 / 1500 / 2000 us |
| LEDC frequency / resolution | 50 Hz / 14 bit |

### ArduPilot-inspired loiter logic (ANCHOR_HOLD)
- Dead zone (loiter radius): 2 m — no throttle inside it.
- P-controller for speed by distance: 0 at the radius, full at max distance.
- Yaw-error speed reduction: speed factor drops linearly to 50% at >= 90 deg heading error.
- Bearing is computed from the current position to the target (great-circle).

### Safety
- GNSS loss in ANCHOR_HOLD: outputs forced to neutral, a warning is logged; the
  mode is kept, so holding resumes when the fix returns.
- Startup ESC/servo test is gated behind `RUN_STARTUP_TEST` (0 by default) — no
  motor spin on reset.

## Control loop
- IMU reads at 20 Hz.
- GNSS navigation solutions at 5 Hz (with a fix; the module drops to ~2.9-3.75 Hz without one).
- Control calculations at 20 Hz.
- Status display at 1 Hz.

## Future
- RC receiver integration (mode switching, manual control).
- Reverse support (1000-1500 us backward).
- Dead zone around neutral (1480-1520 us).
- Full PID tuning for boat dynamics.
- OLED telemetry display.