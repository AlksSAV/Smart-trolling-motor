# TODO

## Build & Env
- [x] `python -m platformio run` — clean build
- [x] Connect board, verify output (flash 16 MB, psram found, USB CDC working)
- [x] Add Adafruit BNO08x and TinyGPSPlus libraries to platformio.ini

## Hardware clarification
- [ ] GPIO of onboard RGB LED and KEY button (marking/schematic)
- [ ] Level of receiver signal (3.3/5 V) — is level shifting needed
- [x] Are I2C pull-ups on BNO085 module (verified: working, address 0x4B)
- [x] UART speed of NEO-M10N (verified: 38400 baud, NMEA + UBX config)

## Firmware (per task)
- [x] ESC: LEDC 50 Hz, 1000–2000 µs, neutral 1500 (GPIO 15, test sequence verified)
- [x] Servo: ±45° (GPIO 16, physical movement verified with IMU feedback)
- [ ] Receiver: read 6 PWM channels (HotRC DS650, pins 4-7, 10-11)
- [x] GNSS: NMEA parse + UBX config (5Hz, Sea model, ~60 sentences/sec)
- [x] IMU: BNO085 read (ARVR Stabilized RV, 20Hz, address 0x4B)
- [x] Holding mode: ArduPilot Loiter Logic (radius, yaw error speed reduction, cascaded PWM)
- [x] Mode switching M/H/S + computeControl() verified on hardware (IMU + GNSS only, no actuators)

## Next Steps
- [ ] Re-verify ESC/servo physical movement after refactor (actuators not connected now)
- [ ] Test GNSS fix outdoors (verify real coordinates and bearing)
- [ ] Test ANCHOR_HOLD with real GPS fix (only simulated target tested so far)
- [ ] Test GNSS-loss guard in ANCHOR_HOLD (force fix loss)
- [ ] Integrate RC receiver (HotRC DS650) for mode switching via PWM channels
- [ ] Add reverse support (1000-1500 µs for backward motion)
- [ ] Add dead zone around neutral (1480-1520 µs = neutral)
- [ ] Tune PID coefficients for boat dynamics (SERVO_P_GAIN, LOITER_RADIUS_M)
- [ ] Add safety features (timeout, max distance, motor failsafe)
- [ ] OLED display for telemetry (optional)