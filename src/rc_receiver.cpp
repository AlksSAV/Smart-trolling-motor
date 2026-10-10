#include "rc_receiver.h"

RCReceiver::RCReceiver() {
    // Инициализация массива пинов согласно config.h
    pins[0] = PIN_RC_CH1;
    pins[1] = PIN_RC_CH2;
    pins[2] = PIN_RC_CH3;
    pins[3] = PIN_RC_CH4;
    pins[4] = PIN_RC_CH5;
    pins[5] = PIN_RC_CH6;
    
    // Начальные значения каналов
    channels.ch1 = RC_PULSE_MID;
    channels.ch2 = RC_PULSE_MID;
    channels.ch3 = RC_PULSE_MID;
    channels.ch4 = RC_PULSE_MID;
    channels.ch5 = RC_PULSE_MID;
    channels.ch6 = RC_PULSE_MID;
}

void RCReceiver::begin() {
    // Настройка всех пинов как входы
    for (int i = 0; i < RC_CHANNEL_COUNT; i++) {
        pinMode(pins[i], INPUT);
    }
}

int RCReceiver::readChannel(uint8_t pin) {
    // Чтение длительности импульса PWM с RC приёмника
    // pulseIn возвращает длительность в микросекундах
    unsigned long duration = pulseIn(pin, HIGH, 25000); // Таймаут 25ms
    
    if (duration == 0) {
        return RC_PULSE_MID; // Если сигнал потерян, возвращаем среднее значение
    }
    
    return (int)duration;
}

void RCReceiver::update() {
    // Последовательное чтение всех каналов
    // Примечание: pulseIn - блокирующая функция, для более продвинутой реализации
    // можно использовать прерывания или библиотеку вроде RCLib
    
    channels.ch1 = readChannel(pins[0]);
    channels.ch2 = readChannel(pins[1]);
    channels.ch3 = readChannel(pins[2]);
    channels.ch4 = readChannel(pins[3]);
    channels.ch5 = readChannel(pins[4]);
    channels.ch6 = readChannel(pins[5]);
}

RCChannels RCReceiver::getChannels() const {
    return channels;
}

int RCReceiver::getChannelValue(uint8_t channelNum) const {
    // channelNum от 1 до 6
    if (channelNum < 1 || channelNum > 6) {
        return RC_PULSE_MID;
    }
    
    switch (channelNum) {
        case 1: return channels.ch1;
        case 2: return channels.ch2;
        case 3: return channels.ch3;
        case 4: return channels.ch4;
        case 5: return channels.ch5;
        case 6: return channels.ch6;
        default: return RC_PULSE_MID;
    }
}

bool RCReceiver::isCentered(uint8_t channelNum, int deadzone) const {
    int value = getChannelValue(channelNum);
    return abs(value - RC_PULSE_MID) <= deadzone;
}