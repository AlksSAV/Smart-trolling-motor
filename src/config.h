#ifndef CONFIG_H
#define CONFIG_H

// Пины согласно pinmap.md
#define PIN_ESC_PWM       15   // LEDC ch0, 50 Hz, 1000-2000 µs
#define PIN_SERVO_PWM     16   // LEDC ch1, 50 Hz, ±45°
#define PIN_RC_CH1        4    // RC канал 1
#define PIN_RC_CH2        5    // RC канал 2
#define PIN_RC_CH3        6    // RC канал 3
#define PIN_RC_CH4        7    // RC канал 4
#define PIN_RC_CH5        10   // RC канал 5
#define PIN_RC_CH6        11   // RC канал 6
#define PIN_GNSS_TX       17   // UART1 TX → GNSS RX
#define PIN_GNSS_RX       18   // UART1 RX ← GNSS TX
#define PIN_IMU_SDA       12   // I2C SDA
#define PIN_IMU_SCL       13   // I2C SCL

// Параметры ESC
#define ESC_FREQ          50      // Частота PWM (Hz)
#define ESC_MIN_US        1000    // Минимальная длительность импульса (µs)
#define ESC_MAX_US        2000    // Максимальная длительность импульса (µs)
#define ESC_NEUTRAL_US    1500    // Нейтральная позиция (µs)

// Параметры сервопривода
#define SERVO_FREQ        50      // Частота PWM (Hz)
#define SERVO_MIN_ANGLE   -45     // Минимальный угол (градусы)
#define SERVO_MAX_ANGLE   45      // Максимальный угол (градусы)

// Параметры RC приёмника
#define RC_CHANNEL_COUNT  6       // Количество каналов
#define RC_PULSE_MIN      1000    // Минимальная длительность импульса (µs)
#define RC_PULSE_MAX      2000    // Максимальная длительность импульса (µs)
#define RC_PULSE_MID      1500    // Средняя длительность импульса (µs)

// Параметры GNSS
#define GNSS_BAUD_RATE    115200  // Скорость UART (проверить!)
#define GNSS_UPDATE_RATE  10      // Частота обновления (Hz)

// Параметры IMU
#define IMU_I2C_ADDR      0x4A    // Адрес BNO085 на шине I2C

// Параметры удержания позиции
#define HOLD_MODE_DEADZONE  2.0   // Мёртвая зона (метры)
#define HOLD_MODE_KP        0.5   // Коэффициент пропорциональности
#define HOLD_MODE_KI        0.1   // Коэффициент интегральной составляющей
#define HOLD_MODE_KD        0.05  // Коэффициент дифференциальной составляющей

#endif // CONFIG_H