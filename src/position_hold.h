#ifndef POSITION_HOLD_H
#define POSITION_HOLD_H

#include <Arduino.h>
#include "config.h"
#include "gnss_module.h"
#include "imu_module.h"
#include "esc_control.h"
#include "servo_control.h"
#include "rc_receiver.h"

enum HoldModeState {
    HOLD_IDLE,        // Режим ожидания
    HOLD_ACTIVE,      // Активное удержание
    HOLD_MANUAL       // Ручное управление
};

class PositionHold {
private:
    GNSSModule* gnss;
    IMUModule* imu;
    ESCControl* esc;
    ServoControl* servo;
    RCReceiver* rc;
    
    HoldModeState state;
    
    // Целевая позиция (точка удержания)
    float targetLat;
    float targetLon;
    bool targetSet;
    
    // PID регуляторы
    float kp, ki, kd;
    float errorIntegral;
    float prevErrorDistance;
    unsigned long lastUpdate;
    
    // Расчёт управляющих воздействий
    void calculateControl(float distance, float bearing);
    
    // Проверка триггеров включения/выключения
    void checkRCCommands();
    
public:
    // Конструктор
    PositionHold(GNSSModule& gnss, IMUModule& imu, 
                 ESCControl& esc, ServoControl& servo, RCReceiver& rc);
    
    // Инициализация
    void begin();
    
    // Основной цикл (вызывать регулярно в loop)
    void update();
    
    // Установить целевую точку (текущая позиция)
    void setTargetPosition();
    
    // Установить целевую точку вручную
    void setTargetPosition(float lat, float lon);
    
    // Переключить режим удержания
    void toggleHoldMode();
    
    // Получить текущее состояние
    HoldModeState getState() const;
    
    // Принудительно выйти из режима удержания
    void disableHold();
};

#endif // POSITION_HOLD_H