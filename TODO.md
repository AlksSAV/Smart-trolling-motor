# TODO

## Build

- [ ] `python -m platformio run` — clean build (prohibited without command — see AGENTS)
- [ ] Connect board, verify output per docs/platformio.md (flash 16 MB, psram found)

## Hardware clarification

- [ ] GPIO of onboard RGB LED and KEY button (marking/schematic)
- [ ] Level of receiver signal (3.3/5 V) — is level shifting needed
- [ ] Are I2C pull-ups on BNO085 module
- [ ] UART speed of NEO-M10N (default 38400 or 115200 — verify by echo)

## Firmware (per task)

- [ ] ESC: LEDC 50 Hz, 1000–2000 µs, neutral 1500
- [ ] Servo: ±45°
- [ ] Receiver: read 6 PWM channels
- [ ] GNSS: NMEA parse
- [ ] IMU: BNO085 read
- [ ] Holding mode: control loop
