#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>
#include "config.h"

class ServoControl {
private:
    uint8_t pin;
    int currentAngle; // Текущий угол поворота (-45 до +45 градусов)
    
    // Преобразует угол в длительность импульса (µs)
    int angleToMicroseconds(int angle);
    
public:
    // Конструктор
    ServoControl(uint8_t servoPin = PIN_SERVO_PWM);
    
    // Инициализация LEDC для PWM
    void begin();
    
    // Установка угла поворота (-45° = влево, 0° = прямо, +45° = вправо)
    void setAngle(int angle);
    
    // Получить текущий угол
    int getAngle() const;
    
    // Центральная позиция (прямо)
    void center();
};

#endif // SERVO_CONTROL_H