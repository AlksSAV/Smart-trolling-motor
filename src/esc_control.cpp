#include "esc_control.h"

ESCControl::ESCControl(uint8_t escPin) : pin(escPin), currentThrottle(0) {
}

void ESCControl::begin() {
    // Настройка LEDC для PWM (новый API)
    ledcAttach(pin, ESC_FREQ, 16);  // Привязываем пин, частота 50 Hz, 16 бит разрешение
    
    // Устанавливаем нейтральную позицию при старте
    stop();
}

int ESCControl::throttleToMicroseconds(int throttle) {
    // Ограничиваем значение тяги от -100 до +100
    throttle = constrain(throttle, -100, 100);
    
    // Преобразуем диапазон [-100, +100] в [ESC_MIN_US, ESC_MAX_US]
    int microseconds = map(throttle, -100, 100, ESC_MIN_US, ESC_MAX_US);
    
    return microseconds;
}

void ESCControl::setThrottle(int throttle) {
    currentThrottle = constrain(throttle, -100, 100);
    int microseconds = throttleToMicroseconds(currentThrottle);
    
    // Устанавливаем длительность импульса через LEDC
    // Для 16-битного разрешения: duty cycle = (microseconds / period) * 65535
    uint32_t period = 1000000 / ESC_FREQ; // Период в микросекундах (20000 для 50Hz)
    uint32_t duty = (microseconds * 65535) / period;
    ledcWrite(pin, duty);
}

int ESCControl::getThrottle() const {
    return currentThrottle;
}

void ESCControl::stop() {
    setThrottle(0);  // Нейтральная позиция (1500 µs)
}