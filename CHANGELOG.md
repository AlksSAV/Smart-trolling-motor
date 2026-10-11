# CHANGELOG
Dates are real. Status: "verified" — what was verified; otherwise — not verified.

## 2026-10-09
- PlatformIO project: `platformio.ini` (env esp32s3: espressif32@6.10.0, arduino, board esp32-s3-devkitc-1 with N16R8 overrides), `src/main.cpp` (flash/PSRAM check). Not verified: build not run.
- Documentation: `docs/pinmap.md` (pin assignment), `docs/platformio.md` (build, commands, verification steps).
- `CHANGELOG.md`, `TODO.md`.
- AGENTS.md: rule "code comments — English only."
- Git: commit `959b2c1` — removed empty skeleton from init commit.

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

## [v0.1.1] - 2026-10-11 (IMU Verified)
### Added
- Real BNO085 IMU data reading via I2C (GPIO 12/13).
- Support for non-standard I2C address `0x4B` (typical for Chinese BNO085 modules).
- ARVR Stabilized Rotation Vector enabled at 20Hz for stable Yaw/Pitch/Roll output.
- `CORE_DEBUG_LEVEL=3` enabled in `build_flags` for Native USB debugging.
### Changed
- Replaced IMU stub with actual working implementation in `src/main.cpp`.
- Updated `TODO.md` and `CHANGELOG.md` to reflect verified hardware status.

## [v0.1.2] - 2026-10-11 (GNSS Optimized)
### Added
- Hybrid GNSS approach: UBX configuration commands sent at startup.
- UBX-CFG-RATE: Increased navigation update rate to 5Hz (200ms) for smoother PID control.
- UBX-CFG-NAV5: Set dynamic model to "Sea" (6) to filter out wave-induced position jumps.
- Automatic UBX checksum calculation helper (`sendUBX`).
### Changed
- Unified IMU + GNSS status output in `main.cpp` for easier debugging.