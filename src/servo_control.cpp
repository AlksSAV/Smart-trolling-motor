#include "servo_control.h"

ServoControl::ServoControl(uint8_t servoPin) : pin(servoPin), currentAngle(0) {
}

void ServoControl::begin() {
    // Настройка LEDC для PWM (новый API)
    ledcAttach(pin, SERVO_FREQ, 16);
    
    // Устанавливаем центральную позицию при старте
    center();
}

int ServoControl::angleToMicroseconds(int angle) {
    // Ограничиваем угол от -45 до +45 градусов
    angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    
    // Преобразуем диапазон [-45, +45] в [1250, 1750] µs
    int microseconds = map(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, 1250, 1750);
    
    return microseconds;
}

void ServoControl::setAngle(int angle) {
    currentAngle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    int microseconds = angleToMicroseconds(currentAngle);
    
    // Устанавливаем длительность импульса через LEDC
    uint32_t period = 1000000 / SERVO_FREQ;
    uint32_t duty = (microseconds * 65535) / period;
    ledcWrite(pin, duty);
}

int ServoControl::getAngle() const {
    return currentAngle;
}

void ServoControl::center() {
    setAngle(0);  // Центральная позиция (прямо)
}