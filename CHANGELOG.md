# CHANGELOG
Dates are real. Status: "verified" — what was verified; otherwise — not verified.

## [Unreleased]
### Removed
- Dead code: unused module classes (`esc_control`, `servo_control`, `gnss_module`, `imu_module`, `position_hold`) and `servo_test.cpp.bak`. `main.cpp` is self-contained; nothing referenced them.
### Changed
- Control calculation extracted into a single `computeControl()`; the control loop and the status display now share it (no duplicated logic).
- ANCHOR_HOLD: on lost GNSS fix the outputs are forced to neutral and a warning is logged, instead of steering on a zero bearing. Mode is kept, so holding resumes when the fix returns.
- Startup ESC/servo test gated behind `RUN_STARTUP_TEST` (0 by default) — no motor spin on reset. Set to 1 for bench testing.
### Not verified
- Build and hardware behavior not yet verified after these changes.

## [v0.3.1] - 2026-10-11 (ESC Test & Simulation Tuning)
### Added
- ESC test sequence in setup(): neutral → arm (1000us) → small throttle (1550us) → neutral
- Servo test sequence: MIN (1000us) → NEUTRAL (1500us) → MAX (2000us) → NEUTRAL
- Simulated anchor target distance increased from 5m to 9m for better ESC range testing
### Verified
- ESC PWM output verified on GPIO 15 (full range 1631-1759 us at 9m simulation)
- Servo PWM output verified on GPIO 16 (full range 1000-2000 us)
- ArduPilot Yaw Error Ratio working correctly: 50% speed at 90°+ heading error
- State Machine transitions: Manual ↔ Heading Hold ↔ Anchor Hold (SIM)

## [v0.3.0] - 2026-10-11 (Full Integration: IMU + GNSS + Servo + ESC + ArduPilot Logic)
### Added
- Unified main.cpp with all subsystems: IMU, GNSS, LEDC outputs, State Machine
- Real LEDC PWM output for Servo (GPIO 16) and ESC (GPIO 15) using ESP32 Core v3.x API
- ArduPilot-inspired Loiter Logic: loiter radius, P-controller for speed, yaw error speed reduction
- Bearing calculation for Anchor Hold mode
- Simulated anchor target ('S' command) for indoor testing without GPS fix
- getCurrentYaw() helper for clean code reuse
### Changed
- LEDC API updated to `ledcAttach()` + `ledcWrite()` (ESP32 Core v3.x compatible, 14-bit)
- Control loop runs at 20 Hz, status display at 1 Hz
- All code comments in English (per AGENTS.md)
### Verified
- Servo physical movement confirmed: responds to IMU yaw changes in real-time
- ESC PWM output verified on GPIO 15 (neutral 1500us)
- ArduPilot Yaw Error Ratio working: speed reduces at large heading errors (50% at 90°+)
- State Machine transitions: Manual ↔ Heading Hold ↔ Anchor Hold (SIM)
- GNSS UBX config: 5Hz + Sea mode active (~60 sentences/sec)

## [v0.2.0] - 2026-10-11 (State Machine & Bearing)
### Added
- State Machine with 3 modes: `MODE_MANUAL`, `MODE_HEADING_HOLD`, `MODE_ANCHOR_HOLD`
- Serial command interface for mode switching ('M', 'H', 'A')
- Bearing calculation function for Anchor Hold (azimuth from current position to target)
- Angle normalization helper for shortest-path rotation
- Real-time error calculation for PID control (yaw error, heading error)
- Protection against Anchor Hold activation without valid GNSS fix
### Changed
- Unified status output showing mode, IMU yaw, GNSS stats, and PID errors
- Improved IMU initialization with retry logic and verbose logging
### Verified
- Heading Hold mode: target yaw locks correctly, error calculation works (tested ±17° deviation)
- Mode transitions: Manual ↔ Heading Hold work seamlessly
- GNSS UBX config: 5Hz + Sea mode active (~60 sentences/sec)

## [v0.1.2] - 2026-10-11 (Sensors Verified & Optimized)
### Added
- Real BNO085 IMU data reading via I2C (GPIO 12/13).
- Support for non-standard I2C address `0x4B` (Chinese BNO085 modules).
- ARVR Stabilized Rotation Vector enabled at 20Hz for stable Yaw output.
- Hybrid GNSS approach: UBX configuration commands sent at startup.
- UBX-CFG-RATE: Increased navigation update rate to 5Hz (200ms) for smoother PID control.
- UBX-CFG-NAV5: Set dynamic model to "Sea" (6) to filter out wave-induced position jumps.
- Robust initialization with retry logic and power-up delays to prevent I2C/UART hangs.
### Changed
- Unified IMU + GNSS status output in `main.cpp` for easier debugging.
- Updated `TODO.md` and `CHANGELOG.md` to reflect verified hardware status.
### Fixed
- Resolved I2C initialization failures caused by simultaneous power-up spikes.

## [v0.1.1] - 2026-10-11 (IMU Verified)
### Added
- Real BNO085 IMU data reading via I2C (GPIO 12/13).
- Support for non-standard I2C address `0x4B` (typical for Chinese BNO085 modules).
- ARVR Stabilized Rotation Vector enabled at 20Hz for stable Yaw/Pitch/Roll output.
- `CORE_DEBUG_LEVEL=3` enabled in `build_flags` for Native USB debugging.
### Changed
- Replaced IMU stub with actual working implementation in `src/main.cpp`.
- Updated `TODO.md` and `CHANGELOG.md` to reflect verified hardware status.

## [v0.1.0] - 2026-10-11
### Added
- Base project structure with PlatformIO (ESP32-S3)
- Modules: ESC Control, Servo Control, RC Receiver, GNSS Module, IMU Module, Position Hold
- NMEA parser for NEO-M10N GNSS receiver
- PID controller framework for position holding
- Debug output via USB CDC (Serial)
### Changed
- Updated LEDC API to support ESP32 Arduino Core v3.x
- Renamed modules for clarity (gnss -> gnss_module, imu -> imu_module)
- Added Adafruit BNO08x library dependency
### Fixed
- Resolved compilation errors related to missing headers and deprecated functions

## 2026-10-09
- PlatformIO project: `platformio.ini` (env esp32s3: espressif32@6.10.0, arduino, board esp32-s3-devkitc-1 with N16R8 overrides), `src/main.cpp` (flash/PSRAM check). Not verified: build not run.
- Documentation: `docs/pinmap.md` (pin assignment), `docs/platformio.md` (build, commands, verification steps).
- `CHANGELOG.md`, `TODO.md`.
- AGENTS.md: rule "code comments — English only."
- Git: commit `959b2c1` — removed empty skeleton from init commit.