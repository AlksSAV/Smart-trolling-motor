# TODO

## Build & Env
- [x] `python -m platformio run` — clean build
- [x] Connect board, verify output (flash 16 MB, psram found)
- [x] Add Adafruit BNO08x library to platformio.ini

## Hardware clarification
- [ ] GPIO of onboard RGB LED and KEY button (marking/schematic)
- [ ] Level of receiver signal (3.3/5 V) — is level shifting needed
- [ ] Are I2C pull-ups on BNO085 module
- [ ] UART speed of NEO-M10N (default 38400 or 115200 — verify by echo)

## Firmware (per task)
- [x] ESC: LEDC 50 Hz, 1000–2000 µs, neutral 1500
- [x] Servo: ±45°
- [x] Receiver: read 6 PWM channels
- [x] GNSS: NMEA parse (basic implementation)
- [x] IMU: BNO085 read (stub/framework ready)
- [x] Holding mode: control loop (PID structure implemented)

## Next Steps
- [ ] Test GNSS fix outdoors
- [ ] Implement real BNO08x data reading (replace stub)
- [ ] Tune PID coefficients for boat dynamics
- [ ] Add safety features (timeout, max distance)
- [ ] Web interface for configuration (optional)