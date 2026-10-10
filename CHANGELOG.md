# CHANGELOG

Dates are real. Status: "verified" — what was verified; otherwise — not verified.

## 2026-10-09

- PlatformIO project: `platformio.ini` (env esp32s3: espressif32@6.10.0,
  arduino, board esp32-s3-devkitc-1 with N16R8 overrides),
  `src/main.cpp` (flash/PSRAM check). Not verified: build not run.
- Documentation: `docs/pinmap.md` (pin assignment), `docs/platformio.md`
  (build, commands, verification steps).
- `CHANGELOG.md`, `TODO.md`.
- AGENTS.md: rule "code comments — English only."
- Git: commit `959b2c1` — removed empty skeleton from init commit.
# Changelog

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