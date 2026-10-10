#include "position_hold.h"
#include <math.h>

PositionHold::PositionHold(GNSSModule& gnss, IMUModule& imu, 
                           ESCControl& esc, ServoControl& servo, RCReceiver& rc)
    : gnss(&gnss), imu(&imu), esc(&esc), servo(&servo), rc(&rc) {
    
    state = HOLD_IDLE;
    targetLat = 0.0;
    targetLon = 0.0;
    targetSet = false;
    
    // PID коэффициенты (настраиваются экспериментально)
    kp = HOLD_MODE_KP;
    ki = HOLD_MODE_KI;
    kd = HOLD_MODE_KD;
    
    errorIntegral = 0.0;
    prevErrorDistance = 0.0;
    lastUpdate = 0;
}

void PositionHold::begin() {
    state = HOLD_IDLE;
    targetSet = false;
    esc->stop();
    servo->center();
    
    Serial.println("[PositionHold] Initialized in IDLE mode");
}

void PositionHold::checkRCCommands() {
    // CH5 - переключение режима удержания (например, тумблер)
    // Если CH5 > 1700 µs - включить удержание
    // Если CH5 < 1300 µs - выключить удержание
    
    int ch5 = rc->getChannelValue(5);
    
    if (ch5 > 1700 && state == HOLD_IDLE) {
        // Включаем режим удержания
        if (gnss->hasFreshData()) {
            setTargetPosition();
            state = HOLD_ACTIVE;
            Serial.println("[PositionHold] HOLD MODE ACTIVATED");
        } else {
            Serial.println("[PositionHold] Cannot activate: no GNSS fix");
        }
    } else if (ch5 < 1300 && state == HOLD_ACTIVE) {
        // Выключаем режим удержания
        disableHold();
        Serial.println("[PositionHold] HOLD MODE DEACTIVATED");
    }
    
    // CH1 и CH2 - ручное управление (газ и поворот)
    // Если мы в режиме HOLD_MANUAL или HOLD_IDLE
    if (state != HOLD_ACTIVE) {
        int ch1 = rc->getChannelValue(1); // Газ
        int ch2 = rc->getChannelValue(2); // Поворот
        
        // Преобразуем PWM (1000-2000) в throttle (-100 до +100)
        int throttle = map(ch1, RC_PULSE_MIN, RC_PULSE_MAX, -100, 100);
        throttle = constrain(throttle, -100, 100);
        
        // Преобразуем PWM в угол серво (-45 до +45)
        int angle = map(ch2, RC_PULSE_MIN, RC_PULSE_MAX, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
        angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
        
        esc->setThrottle(throttle);
        servo->setAngle(angle);
        
        state = HOLD_MANUAL;
    }
}

void PositionHold::calculateControl(float distance, float bearing) {
    unsigned long now = millis();
    float dt = (now - lastUpdate) / 1000.0; // Время в секундах
    lastUpdate = now;
    
    if (dt <= 0 || dt > 1.0) {
        dt = 0.1; // Защита от деления на ноль и слишком больших интервалов
    }
    
    // Ошибка расстояния (целевая точка = 0 метров)
    float error = -distance; // Отрицательная, т.к. хотим уменьшить расстояние
    
    // PID регулятор
    errorIntegral += error * dt;
    errorIntegral = constrain(errorIntegral, -50, 50); // Anti-windup
    
    float derivative = (error - prevErrorDistance) / dt;
    prevErrorDistance = error;
    
    float pidOutput = kp * error + ki * errorIntegral + kd * derivative;
    
    // Преобразуем PID output в тягу (-100 до +100)
    int throttle = constrain((int)(pidOutput * 10), -100, 100);
    
    // Расчёт угла поворота
    // Получаем текущий курс от компаса
    float currentHeading = imu->getHeading();
    
    // Разница между целевым азимутом и текущим курсом
    float headingError = bearing - currentHeading;
    
    // Нормализация угла в диапазон [-180, +180]
    while (headingError > 180) headingError -= 360;
    while (headingError < -180) headingError += 360;
    
    // Пропорциональный регулятор для угла
    int servoAngle = constrain((int)(headingError * 0.5), SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    
    // Применяем управления
    esc->setThrottle(throttle);
    servo->setAngle(servoAngle);
    
    // Отладка
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 1000) {
        Serial.printf("[PID] Dist=%.1fm Bear=%.1f° Head=%.1f° Thr=%d%% Srv=%d°\n",
                      distance, bearing, currentHeading, throttle, servoAngle);
        lastPrint = millis();
    }
}

void PositionHold::update() {
    checkRCCommands();
    
    if (state == HOLD_ACTIVE) {
        // Проверяем наличие свежих данных GNSS
        if (!gnss->hasFreshData()) {
            Serial.println("[PositionHold] WARNING: Lost GNSS signal!");
            return;
        }
        
        GNSSData gps = gnss->getData();
        
        // Если целевая точка не установлена, устанавливаем текущую позицию
        if (!targetSet) {
            setTargetPosition(gps.latitude, gps.longitude);
        }
        
        // Рассчитываем расстояние и азимут до цели
        float distance = gnss->distanceTo(targetLat, targetLon);
        float bearing = gnss->bearingTo(targetLat, targetLon);
        
        // Проверяем мёртвую зону
        if (distance < HOLD_MODE_DEADZONE) {
            // Мы в пределах допустимой зоны, останавливаем мотор
            esc->stop();
            servo->center();
            
            static bool printed = false;
            if (!printed) {
                Serial.println("[PositionHold] Within deadzone - holding position");
                printed = true;
            }
        } else {
            // Нужно двигаться к цели
            calculateControl(distance, bearing);
        }
    }
}

void PositionHold::setTargetPosition() {
    GNSSData gps = gnss->getData();
    if (gps.isValid) {
        setTargetPosition(gps.latitude, gps.longitude);
    } else {
        Serial.println("[PositionHold] ERROR: Cannot set target - no GNSS fix");
    }
}

void PositionHold::setTargetPosition(float lat, float lon) {
    targetLat = lat;
    targetLon = lon;
    targetSet = true;
    
    // Сброс PID интегратора
    errorIntegral = 0.0;
    prevErrorDistance = 0.0;
    lastUpdate = millis();
    
    Serial.printf("[PositionHold] Target set: Lat=%.6f Lon=%.6f\n", lat, lon);
}

void PositionHold::toggleHoldMode() {
    if (state == HOLD_IDLE || state == HOLD_MANUAL) {
        if (gnss->hasFreshData()) {
            setTargetPosition();
            state = HOLD_ACTIVE;
            Serial.println("[PositionHold] HOLD MODE ACTIVATED");
        } else {
            Serial.println("[PositionHold] Cannot activate: no GNSS fix");
        }
    } else if (state == HOLD_ACTIVE) {
        disableHold();
        Serial.println("[PositionHold] HOLD MODE DEACTIVATED");
    }
}

HoldModeState PositionHold::getState() const {
    return state;
}

void PositionHold::disableHold() {
    state = HOLD_IDLE;
    targetSet = false;
    esc->stop();
    servo->center();
    
    // Сброс PID
    errorIntegral = 0.0;
    prevErrorDistance = 0.0;
}