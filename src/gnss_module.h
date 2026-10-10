#ifndef GNSS_MODULE_H
#define GNSS_MODULE_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include "config.h"

struct GNSSData {
    bool isValid;       // Флаг валидности данных
    float latitude;     // Широта (градусы)
    float longitude;    // Долгота (градусы)
    float altitude;     // Высота над уровнем моря (метры)
    float speed;        // Скорость (км/ч)
    float course;       // Курс (градусы, 0-360)
    uint8_t satellites; // Количество видимых спутников
    uint8_t fixQuality; // Качество фиксации (0=нет, 1=GPS, 2=DGPS)
    unsigned long lastUpdate; // Время последнего обновления (millis)
};

class GNSSModule {
private:
    HardwareSerial* gpsSerial;
    GNSSData data;
    
    // Парсинг NMEA предложений
    void parseGGA(const char* sentence);
    void parseRMC(const char* sentence);
    void parseVTG(const char* sentence);
    
    // Вспомогательная функция для парсинга полей
    char* getNextField(char* str);
    
public:
    // Конструктор
    GNSSModule(HardwareSerial& serial = Serial1);
    
    // Инициализация UART и GNSS модуля
    void begin();
    
    // Обновление данных (вызывать регулярно в loop)
    void update();
    
    // Получить структуру с данными
    GNSSData getData() const;
    
    // Проверка наличия свежих данных (не старше 2 секунд)
    bool hasFreshData(unsigned long maxAgeMs = 2000) const;
    
    // Расстояние до точки (в метрах)
    float distanceTo(float targetLat, float targetLon) const;
    
    // Азимут к точке (в градусах)
    float bearingTo(float targetLat, float targetLon) const;
};

#endif // GNSS_MODULE_H