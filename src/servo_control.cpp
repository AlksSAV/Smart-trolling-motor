#include "servo_control.h"

ServoControl::ServoControl(uint8_t servoPin) : pin(servoPin), currentAngle(0) {
}

void ServoControl::begin() {
    // Настройка LEDC для PWM (канал 1, чтобы не конфликтовать с ESC)
    ledcSetup(1, SERVO_FREQ, 16);  // Канал 1, частота 50 Hz, 16 бит разрешение
    ledcAttachPin(pin, 1);          // Привязываем пин к каналу 1
    
    // Устанавливаем центральную позицию при старте
    center();
}

int ServoControl::angleToMicroseconds(int angle) {
    // Ограничиваем угол от -45 до +45 градусов
    angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    
    // Преобразуем диапазон [-45, +45] в [1000, 2000] µs
    // Стандартный серво: 1000 µs = -90°, 1500 µs = 0°, 2000 µs = +90°
    // Но у нас диапазон только ±45°, поэтому:
    // -45° → ~1250 µs
    //   0° → 1500 µs
    // +45° → ~1750 µs
    
    int microseconds = map(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, 1250, 1750);
    
    return microseconds;
}

void ServoControl::setAngle(int angle) {
    currentAngle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    int microseconds = angleToMicroseconds(currentAngle);
    
    // Устанавливаем длительность импульса через LEDC
    uint32_t duty = (microseconds * 65536) / (1000000 / SERVO_FREQ);
    ledcWrite(1, duty);
}

int ServoControl::getAngle() const {
    return currentAngle;
}

void ServoControl::center() {
    setAngle(0);  // Центральная позиция (прямо)
}