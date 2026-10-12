# TODO

## Build & Env
- [x] `python -m platformio run` — clean build
- [x] Connect board, verify output (flash 16 MB, psram found, USB CDC working)
- [x] Add Adafruit BNO08x and TinyGPSPlus libraries to platformio.ini

## Hardware clarification
- [ ] GPIO of onboard RGB LED and KEY button (marking/schematic)
- [ ] Level of receiver signal (3.3/5 V) — is level shifting needed
- [x] Are I2C pull-ups on BNO085 module (verified: working, address 0x4B)
- [x] UART speed of UBX-M10050 (verified: 38400 baud, NMEA + UBX config)

## Firmware (per task)
- [x] ESC: LEDC 50 Hz, 1000–2000 µs, neutral 1500 (GPIO 15, test sequence verified)
- [x] Servo: ±45° (GPIO 16, physical movement verified with IMU feedback)
- [ ] Receiver: read 6 PWM channels (HotRC DS650, pins 4-7, 10-11)
- [x] GNSS: NMEA parse (GGA+RMC) + конфиг модуля через UBX (5 Гц, dynModel SEA, 38400, вывод 2 предложения/цикл)
  - Модуль настроен через `tools/gnss_config.py`; `dynModel` был `6 (AIR1)`, не Sea — прежняя запись «Sea model» в CHANGELOG была неверной.
- [x] IMU: BNO085 read (ARVR Stabilized RV, 20Hz, address 0x4B)
- [x] Holding mode: ArduPilot Loiter Logic (радиус 2 м, скорость по дистанции; снижение скорости по ошибке курса убрано 2026-10-12 — вместо него реверс)
- [x] Mode switching M/H/S + computeControl() verified on hardware (IMU + GNSS only, no actuators)

## Next Steps
- [x] GNSS: power-cycle — конфигурация (measRate 200, SEA, GGA+RMC) сохраняется во Flash (2026-10-12)
- [ ] GNSS: замерить поток финального набора (2 предложения) при наличии фикса — без фикса в помещении замерено 10 msg/s
- [ ] GNSS: выяснить, почему `CFG-VALGET` для слоя BBR возвращает UBX-NAK
- [ ] IMU: проверить срабатывание защиты по устаревшим данным (отключить SDA/SCL в H/S, ждать `[WARN] IMU data stale` через ~1 с)
- [x] Reverse support: ESC 1450..1250 µs назад, 1550..1800 вперёд, ANCHOR_HOLD переключает FWD/REV с гистерезисом 100°/80° — расчёт проверен по логу в помещении (режим `S`), актуаторы не подключены (2026-10-12)
- [x] Dead zone around neutral: учтена как ±50 µs (1450-1550) в `speedToEscUs()`; ни одного значения внутри полосы, кроме 1500 (по логу)
- [x] HEADING_HOLD (вариант «а»): серво ±90° в сторону ошибки, тяга растёт с ошибкой — расчёт проверен по логу, актуаторы не подключены
- [ ] ANCHOR_HOLD: ограничить `servoAngle` в `computeControl()` до ±90° (в логе сейчас бывает −96.2°, а выход уже обрезан)
- [ ] ANCHOR_HOLD: сузить гистерезис реверса (например 95°/85°), чтобы ошибка направления у ±90° не достигала 10°; подобрать на воде
- [x] Сверить yaw с компасом: ось платы вперёд на север и на восток — знак счёта определён (против часовой стрелки), исправлен через `getCurrentHeading()` и проверен на железе (2026-10-12)
- [ ] ANCHOR_HOLD: определить сдвиг heading относительно истинного азимута на воде после калибровки «восьмёркой» (в помещении неразделим: монтаж + склонение + неоткалиброванный магнитометр; сдвиг ~242°)
- [ ] Магнитометр BNO085 в покое не докалиброван (yaw дрейфует ~0.07°/с); калибровать вращением на воде
- [ ] Знак серво (`SERVO_DIR`): положительная ошибка курса → 2000 µs, повернёт ли нос в нужную сторону — проверить на стенде
- [ ] Армирование ESC: 3 с на 1500 µs сразу после старта, до инициализации IMU/GNSS. Сейчас ШИМ 1500 появляется через ~2–4 с (пропущено по решению владельца)
- [ ] Стенд с ESC и серво (винт снят): порог трогания ESC, реальное направление серво, время поворота на 180° при смене FWD/REV, скачок серво между +90° и −90° в HEADING_HOLD на ошибках ±5° и ±180°
- [ ] HEADING_HOLD: тяга при выходе из мёртвой зоны 5° возникает ступенью (1500 → ~1564); решить, нужна ли плавная шкала
- [ ] Заменить серво на 180° (±90°); до замены ±90° в прошивке соответствует диапазону импульса, а не реальному углу текущего серво (±45°)
- [ ] IMU рядом с мотором (≤0.5 м): проверить помехи магнитометру от мотора и кабеля 48 В (вращать лодку при разном токе, сравнить с компасом); ось платы вперёд/вверх — формула `getCurrentYaw()` рассчитана на горизонтальное положение
- [ ] Test GNSS fix outdoors (verify real coordinates and bearing)
- [ ] Test ANCHOR_HOLD with real GPS fix (only simulated target tested so far)
- [ ] Test GNSS-loss guard in ANCHOR_HOLD (force fix loss)
- [ ] Integrate RC receiver (HotRC DS650) for mode switching via PWM channels; реверс в MANUAL получится сам: 1000-1500 µs назад, 1500-2000 µs вперёд
- [ ] Tune PID coefficients for boat dynamics (HEADING_* параметры, LOITER_RADIUS_M)
- [ ] Реверс на воде: проверить поведение лодки при тяге назад с носа (гипотеза о неустойчивости по курсу не проверена), начать с малой мощности
- [ ] Add safety features (timeout, max distance, motor failsafe)
- [ ] OLED display for telemetry (optional)