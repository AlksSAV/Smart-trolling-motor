# TODO

## Build & Env
- [x] `python -m platformio run` — clean build
- [x] Connect board, verify output (flash 16 MB, psram found)
- [x] Add Adafruit BNO08x library to platformio.ini

## Hardware clarification
- [ ] GPIO of onboard RGB LED and KEY button (marking/schematic)
- [ ] Level of receiver signal (3.3/5 V) — is level shifting needed
- [x] Are I2C pull-ups on BNO085 module (verified: working, Chinese module uses address 0x4B)
- [x] UART speed of NEO-M10N (verified: 38400 baud, NMEA protocol active)

## Firmware (per task)
- [x] ESC: LEDC 50 Hz, 1000–2000 µs, neutral 1500
- [x] Servo: ±45°
- [x] Receiver: read 6 PWM channels
- [x] GNSS: NMEA parse + UBX config injection (5Hz update rate, Sea dynamic model)
- [x] IMU: BNO085 read (real data reading implemented, ARVR Stabilized RV, 20Hz, address 0x4B)
- [x] Holding mode: control loop (PID structure implemented)

## Next Steps
- [ ] Test GNSS fix outdoors and verify UART1 communication
- [ ] Integrate real IMU Yaw data into the PID holding loop
- [ ] Tune PID coefficients for boat dynamics
- [ ] Add safety features (timeout, max distance)
- [ ] Web interface for configuration (optional)