#ifndef RC_RECEIVER_H
#define RC_RECEIVER_H

#include <Arduino.h>
#include "config.h"

struct RCChannels {
    int ch1; // Обычно: газ/тяга
    int ch2; // Обычно: поворот/руль
    int ch3; // Вспомогательный 1
    int ch4; // Вспомогательный 2
    int ch5; // Вспомогательный 3 (например, включение режима удержания)
    int ch6; // Вспомогательный 4
};

class RCReceiver {
private:
    uint8_t pins[RC_CHANNEL_COUNT];
    RCChannels channels;
    
    // Чтение одного канала (в микросекундах)
    int readChannel(uint8_t pin);
    
public:
    // Конструктор
    RCReceiver();
    
    // Инициализация пинов
    void begin();
    
    // Обновление значений всех каналов
    void update();
    
    // Получить структуру со значениями каналов
    RCChannels getChannels() const;
    
    // Получить значение конкретного канала (1-6)
    int getChannelValue(uint8_t channelNum) const;
    
    // Проверка, находится ли канал в средней позиции (с учетом мертвой зоны)
    bool isCentered(uint8_t channelNum, int deadzone = 50) const;
};

#endif // RC_RECEIVER_H