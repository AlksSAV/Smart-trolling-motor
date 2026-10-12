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
| ESC | Flycolor 90A marine, bidirectional PWM | LEDC 50 Hz | GPIO 15 |
| Steering servo | digital PWM, 1000-2000 us; firmware assumes +/-90 deg (180 deg servo planned, current one is +/-45 deg) | LEDC 50 Hz | GPIO 16 |
| RC receiver | HotRC DS650 (6 ch) | GPIO input | 4-7, 10-11 (not integrated) |

Pin assignment: `docs/pinmap.md` (single source). Equipment registry: `docs/equipment.md`.

## Verified facts (on real hardware)
- BNO085 I2C address is `0x4B` (non-standard, typical for Chinese modules).
- UBX-M10050 runs at `38400 baud` (not 115200, not 9600), firmware `ROM SPG 5.10 (7b202e)`.
- Module configuration is stored in the module's Flash (written with
  `tools/gnss_config.py --set`, UBX-ACK received) and survives a power cycle:
  `measRate` 200 ms (5 Hz), `dynModel` SEA (5), NMEA output on UART1 only `GGA` + `RMC`.
  Verified 2026-10-12: after unplugging power, both RAM and Flash layers read back
  these values. The BBR layer cannot be read (UBX-NAK on `CFG-VALGET`).
- The firmware does not write to the GNSS module. Earlier firmware re-sent legacy
  `UBX-CFG-RATE` / `UBX-CFG-NAV5` (with `dynModel 6` = AIR1) on every boot; that
  masked the real state (Flash had no `measRate`, RAM showed AIR1). Removed.
- BNO085 yaw grows counter-clockwise; GNSS bearing and servo angle grow clockwise.
  Verified on hardware 2026-10-12: one full clockwise turn gave -360 deg of yaw
  (N 117.8, E 36.8, S 293.1, W 198.7, back to N; sum of steps exactly -360).
  `getCurrentHeading()` = (360 - yaw) mirrors it into the bearing convention;
  `getCurrentYaw()` keeps the raw sensor value. All control uses the heading.
- Yaw/heading against the phone compass and true north (2026-10-12, indoors):
  - N: yaw 114.4, heading 245.6  |  E: yaw 329.0, heading 30.7
  - one continuous run (no reset between points): E yaw 13.0 / heading 347.9,
    N yaw 118.0 / heading 242.1 -> heading grows clockwise, the fix is correct.
  - A constant offset between heading and true azimuth (~242 deg) mixes the
    mounting, the magnetic declination and the uncalibrated magnetometer; it is
    NOT applied. Indoor magnetic distortion dominates (the 90 deg turn measured
    106 deg, E point scattered 337-353). Determine it on the water after a
    figure-eight calibration.
- Correction: earlier docs claimed "Sea dynamic model active"; the module actually
  ran `dynModel 6 (AIR1)` until it was set to SEA on 2026-10-11.
- Measured 2026-10-12 with the stored configuration, no fix (indoors, `Fix: NO`):
  10 msg/s (2 sentences x 5 Hz). The earlier 3.18-3.76 Hz without a fix was not
  reproduced; its cause is unknown. Rate with a fix has not been re-measured.
- LEDC API (ESP32 Core v3.x): `ledcAttach(pin, freq, resolution)` + `ledcWrite(pin, duty)`.
- PWM: 14-bit resolution (16384 steps), 50 Hz = 20 ms period.

## Software

Main firmware in `src/main.cpp` (self-contained). Heading EKF in `src/ekf_heading.h/.cpp`.

### Operating modes
1. **MANUAL** — ESC and servo at neutral. (RC control is a future task.)
2. **HEADING_HOLD** — the boat is under way and holds its course. The servo turns
   fully (+/-90 deg) to the side of the heading error, ESC thrust (forward only)
   grows with the error. Not a standstill mode.
3. **ANCHOR_HOLD** — GNSS holds the position; both ESC and servo active, thrust
   forward or reverse.

Mode is switched by serial command: `M` (manual), `H` (heading hold),
`A` (anchor hold, requires a GNSS fix), `S` (anchor hold with a simulated target
for indoor testing). `L` toggles the 20 Hz CSV tuning log (prefix `LOG,`).

### Heading EKF (2 states)

State: `[yaw, gyro_bias]` — heading и накопленный дрейф магнитометра (deg/s).
Матрицы 2×2 написаны вручную, без зависимостей. Источники: `src/ekf_heading.h/.cpp`.

| Параметр | Значение (заглушка) |
|----------|-------------------|
| `qYawDeg2` | 0.001 |
| `qBiasDeg2PerS2` | 1e-5 |
| `rCOG` | 10° (1-sigma) |
| COG speed threshold | 0.3 m/s |
| Innovation gate | 60° |

**Принцип работы:**
- `predict()` на каждый вектор поворота IMU (20 Гц): `yaw += delta_yaw - bias * dt`.
- `correct()` на каждое решение GNSS (5 Гц): подаётся **только COG** из RMC.
  - Bearing к цели — не measurement heading, игнорируется.
  - COG отбрасывается: скорость < 0.3 м/с; ANCHOR_HOLD + реверс (COG указывает на 180° от носа).
- Первое измерение `seedHeading()`: yaw жёстко ставится в рамку GNSS (истинный север). Bias не трогается, на следующих циклах коррекция сходится.
- Gate 60°: измерения дальше отклоняются, счётчик в CSV-логе.

**Тюнинг Q/R** — не сделан. Требуется CSV-лог (`L`) и анализ на ПК.

> **Статус:** не проверено. Скомпилировано (`pio run` — RAM 8.4%, Flash 6.0%), но плата недоступна. Не запускать на воде до подбора Q/R.

### Control parameters (`src/main.cpp`)
| Parameter | Value |
|-----------|-------|
| Loiter radius (dead zone) | 2.0 m |
| Max distance (full speed) | 10.0 m |
| ESC neutral | 1500 us |
| ESC dead band (ESC ignores it) | +/-50 us (1450..1550) |
| ESC max forward / max reverse | 1800 / 1250 us |
| Servo min / neutral / max | 1000 / 1500 / 2000 us = -90 / 0 / +90 deg (linear) |
| Servo direction `SERVO_DIR` | +1 (sign not verified on the bench) |
| Reverse enter / exit (off the bow) | 100 / 80 deg |
| HEADING_HOLD dead zone / full thrust / max speed | 5 deg / 45 deg / 50% |
| LEDC frequency / resolution | 50 Hz / 14 bit |

HEADING_HOLD values are starting values, not tuned.

### Output mapping
- `speedToEscUs(-1..+1)`: speed 0 = 1500; positive maps to 1550..1800, negative to
  1450..1250, so every non-zero command is outside the ESC dead band.
- `angleToServoUs(-90..+90 deg)` = 1500 + `SERVO_DIR` * angle * 500 / 90.
- The ESC is a Flycolor 90A marine ESC, bidirectional, already calibrated 1000/1500/2000;
  it needs 3 s of 1500 us after power-up to arm and stops by itself on signal loss
  (per the owner; not in a manual). The firmware does not arm it explicitly (see TODO).

### ArduPilot-inspired loiter logic (ANCHOR_HOLD)
- Dead zone (loiter radius): 2 m — no throttle inside it.
- P-controller for speed by distance: 0 at the radius, full at max distance.
- Bearing is computed from the current position to the target (great-circle).
- Direction to the target relative to the bow `d` is in -180..+180 deg. Thrust goes
  forward with the servo at `d`; when the target is more than 100 deg off the bow it
  goes backward with the servo turned 180 deg away (`d -/+ 180`), so the reverse
  thrust still points at the target. It returns to forward below 80 deg (hysteresis).
- The old yaw-error speed reduction (down to 50% at 90 deg) was removed: it existed
  only because the motor could not push backward.
- Known limits: near +/-90 deg the servo hits its stop before the thrust direction
  switches (up to 10 deg pointing error inside the hysteresis band); the logged
  `servoAngle` is not clamped to +/-90 deg while the servo output is.

### Safety
- GNSS loss in ANCHOR_HOLD: outputs forced to neutral, a warning is logged; the
  mode is kept, so holding resumes when the fix returns.
- IMU stale/absent in HEADING_HOLD or ANCHOR_HOLD (no rotation vector for
  `IMU_TIMEOUT_MS` = 1000 ms, or none received yet since boot): outputs forced to
  neutral, a warning is logged, and `H` is refused. Holding resumes when data returns.
  Verified: `H` right after reset takes the real yaw, not 0 deg. The 1000 ms
  timeout itself (cable pulled) has not been tested.
- Startup ESC/servo test is gated behind `RUN_STARTUP_TEST` (0 by default) — no
  motor spin on reset. It no longer sends 1000 us (full reverse on a bidirectional
  ESC); the ESC step is 1600 us forward.

## Control loop
- IMU reads at 20 Hz.
- GNSS navigation solutions at 5 Hz (`measRate` 200 ms stored in module Flash).
- Control calculations at 20 Hz.
- EKF `predict()` on each IMU rotation vector; `correct()` on each valid COG (5 Hz when moving).
- Status display at 1 Hz.

## Future
- **EKF tuning:** collect CSV log (`L`) on the water, fit Q/R with `tools/log_parse.py`
  (parser not written yet). Current Q/R are placeholders.
- EKF: verify `seedHeading` on the first COG (true-north frame). On standstill the
  constant heading offset remains — no COG, no correction.
- RC receiver integration (mode switching, manual control; 1000-1500 us = reverse,
  1500-2000 us = forward).
- Magnetic declination: `seedHeading` puts the heading into the true-north frame on
  the first valid COG, but the offset is not separated into mounting, declination
  and magnetometer bias, and it is not available at standstill. The yaw direction
  against a compass is verified (counter-clockwise, mirrored by
  `getCurrentHeading()`), the constant offset is not determined.
- Full PID tuning for boat dynamics.
- OLED telemetry display.