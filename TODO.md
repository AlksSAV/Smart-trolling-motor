# TODO

## Build & Env
- [x] `python -m platformio run` — clean build
- [x] Connect board, verify output (flash 16 MB, psram found, USB CDC working)
- [x] Add Adafruit BNO08x and TinyGPSPlus libraries to platformio.ini

## Hardware clarification
- [ ] GPIO of onboard RGB LED and KEY button (marking/schematic)
- [ ] Level of receiver signal (3.3/5 V) — is level shifting needed
- [x] Are I2C pull-ups on BNO085 module (verified: working, Chinese module uses address 0x4B)
- [x] UART speed of NEO-M10N (verified: 38400 baud, NMEA protocol active, UBX config applied)

## Firmware (per task)
- [ ] ESC: LEDC 50 Hz, 1000–2000 µs, neutral 1500
- [ ] Servo: ±45°
- [ ] Receiver: read 6 PWM channels (HotRC DS650)
- [x] GNSS: NMEA parse + UBX config injection (5Hz update rate, Sea dynamic model, ~30 sentences/sec verified)
- [x] IMU: BNO085 read (real data implemented, ARVR Stabilized RV, 20Hz, address 0x4B)
- [ ] Holding mode: control loop (PID structure needs integration with real sensor data)

## Next Steps
- [ ] Test GNSS fix outdoors (verify real coordinates and speed)
- [ ] Integrate real IMU Yaw and GNSS data into the PID holding loop
- [ ] Tune PID coefficients for boat dynamics
- [ ] Add safety features (timeout, max distance)
- [ ] Web interface for configuration (optional)