#ifndef ESC_CONTROL_H
#define ESC_CONTROL_H

#include <Arduino.h>
#include "config.h"

class ESCControl {
private:
    uint8_t pin;
    int currentThrottle; // Текущее значение тяги (-100 до +100)
    
    // Преобразует значение тяги в длительность импульса (µs)
    int throttleToMicroseconds(int throttle);
    
public:
    // Конструктор
    ESCControl(uint8_t escPin = PIN_ESC_PWM);
    
    // Инициализация LEDC для PWM
    void begin();
    
    // Установка тяги (-100 = полный назад, 0 = нейтраль, +100 = полный вперед)
    void setThrottle(int throttle);
    
    // Получить текущее значение тяги
    int getThrottle() const;
    
    // Нейтральная позиция (стоп)
    void stop();
};

#endif // ESC_CONTROL_H