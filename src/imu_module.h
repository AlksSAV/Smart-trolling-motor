#ifndef IMU_MODULE_H
#define IMU_MODULE_H

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

struct IMUData {
    bool isValid;       // Флаг валидности данных
    float heading;      // Курс/азимут (градусы, 0-360, 0=North)
    float pitch;        // Тангаж (градусы)
    float roll;         // Крен (градусы)
    float yaw;          // Рыскание (градусы)
    float accuracy;     // Точность измерения (0-3, где 3 - highest)
    unsigned long lastUpdate; // Время последнего обновления (millis)
};

class IMUModule {
private:
    TwoWire* i2cBus;
    uint8_t i2cAddr;
    IMUData data;
    
    // Инициализация BNO085 (упрощенная, без полной библиотеки Adafruit)
    bool initBNO085();
    
    // Чтение данных из регистров (базовая реализация)
    bool readSensorData();
    
public:
    // Конструктор
    IMUModule(TwoWire& wire = Wire, uint8_t addr = IMU_I2C_ADDR);
    
    // Инициализация I2C и сенсора
    void begin();
    
    // Обновление данных (вызывать регулярно в loop)
    void update();
    
    // Получить структуру с данными
    IMUData getData() const;
    
    // Проверка наличия свежих данных
    bool hasFreshData(unsigned long maxAgeMs = 100) const;
    
    // Получить только курс (heading)
    float getHeading() const;
};

#endif // IMU_MODULE_H