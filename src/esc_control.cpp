#include "esc_control.h"

ESCControl::ESCControl(uint8_t escPin) : pin(escPin), currentThrottle(0) {
}

void ESCControl::begin() {
    // Настройка LEDC для PWM
    ledcSetup(0, ESC_FREQ, 16);  // Канал 0, частота 50 Hz, 16 бит разрешение
    ledcAttachPin(pin, 0);        // Привязываем пин к каналу 0
    
    // Устанавливаем нейтральную позицию при старте
    stop();
}

int ESCControl::throttleToMicroseconds(int throttle) {
    // Ограничиваем значение тяги от -100 до +100
    throttle = constrain(throttle, -100, 100);
    
    // Преобразуем диапазон [-100, +100] в [ESC_MIN_US, ESC_MAX_US]
    // -100 → 1000 µs (полный назад)
    //   0  → 1500 µs (нейтраль)
    // +100 → 2000 µs (полный вперед)
    int microseconds = map(throttle, -100, 100, ESC_MIN_US, ESC_MAX_US);
    
    return microseconds;
}

void ESCControl::setThrottle(int throttle) {
    currentThrottle = constrain(throttle, -100, 100);
    int microseconds = throttleToMicroseconds(currentThrottle);
    
    // Устанавливаем длительность импульса через LEDC
    // Для 16-битного разрешения: 0-65535 соответствует 0-100% duty cycle
    // Нам нужно преобразовать микросекунды в duty cycle
    uint32_t duty = (microseconds * 65536) / (1000000 / ESC_FREQ);
    ledcWrite(0, duty);
}

int ESCControl::getThrottle() const {
    return currentThrottle;
}

void ESCControl::stop() {
    setThrottle(0);  // Нейтральная позиция (1500 µs)
}