# Pin Map

Firmware pin assignment for ESP32-S3-N16R8. User wires per this table.
Unassigned pins: "not verified" — no data, no numbers.

## Restrictions of the ESP32-S3-N16R8 chip

| GPIO | Reason |
|------|--------|
| 26–32 | SPI flash |
| 33–37 | Octal PSRAM (N16R8) |
| 19, 20 | USB D-/D+ |
| 43, 44 | UART0 (console) |
| 45, 46 | Strapping (VDD_SPI) |
| 0, 3 | Strapping (BOOT, JTAG) — unused |

## Assignment

| Function | GPIO | Periphery | Notes |
|----------|------|-----------|-------|
| ESC PWM (50 Hz, 1000–2000 µs) | 15 | LEDC ch0 | Neutral 1500 µs |
| Steering servo (PWM 50 Hz) | 16 | LEDC ch1 | ±45° |
| RC CH1 | 4 | GPIO input | HotRC DS650, individual PWM per channel |
| RC CH2 | 5 | GPIO input | |
| RC CH3 | 6 | GPIO input | |
| RC CH4 | 7 | GPIO input | |
| RC CH5 | 10 | GPIO input | |
| RC CH6 | 11 | GPIO input | |
| GNSS NEO-M10N RX← | 17 | UART1 TX | ESP→GNSS |
| GNSS NEO-M10N →RX | 18 | UART1 RX | GNSS→ESP |
| IMU BNO085 SDA | 12 | I2C SDA | Pull-ups: verify module |
| IMU BNO085 SCL | 13 | I2C SCL | Pull-ups: verify module |
| RGB LED | not verified | — | Onboard LED, GPIO unknown |
| KEY button | not verified | — | GPIO unknown |

Power is separate from the power part. Board powered from DC-DC with margin.
All signals are 3.3 V level. 5 V devices (ESC, servo, receiver):
level shifting question — open (see TODO).
