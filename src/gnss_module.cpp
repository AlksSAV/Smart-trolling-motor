#include "gnss_module.h"
#include <math.h>

GNSSModule::GNSSModule(HardwareSerial& serial) : gpsSerial(&serial) {
    data.isValid = false;
    data.latitude = 0.0;
    data.longitude = 0.0;
    data.altitude = 0.0;
    data.speed = 0.0;
    data.course = 0.0;
    data.satellites = 0;
    data.fixQuality = 0;
    data.lastUpdate = 0;
}

void GNSSModule::begin() {
    // Инициализация UART1 для связи с NEO-M10N
    gpsSerial->begin(GNSS_BAUD_RATE, SERIAL_8N1, PIN_GNSS_RX, PIN_GNSS_TX);
    
    // Небольшая задержка для стабилизации модуля
    delay(100);
    
    // Опционально: можно отправить команды конфигурации для увеличения частоты обновления
    // Например, установить 10 Hz вместо стандартных 1 Hz
    // gpsSerial->println("$PUBX,40,GGA,0,10,0,0,0,0*5C"); // GGA каждые 10 сек? Нет, нужно чаще
    // Для NEO-M10N лучше использовать UBX протокол, но начнем с NMEA
}

char* GNSSModule::getNextField(char* str) {
    char* p = strchr(str, ',');
    if (p) {
        *p = '\0';
        return p + 1;
    }
    return NULL;
}

void GNSSModule::parseGGA(const char* sentence) {
    // $GNGGA или $GPGGA
    // Поля: Time, Lat, N/S, Lon, E/W, Quality, Satellites, HDOP, Alt, M, Geoid, M, Age, DiffStation, Checksum
    
    char buffer[120];
    strncpy(buffer, sentence, sizeof(buffer));
    buffer[sizeof(buffer) - 1] = '\0';
    
    char* ptr = buffer;
    ptr = getNextField(ptr); // Time
    
    // Latitude
    char* latStr = ptr;
    ptr = getNextField(ptr);
    if (latStr && strlen(latStr) > 0) {
        float lat = atof(latStr);
        // Формат DDMM.MMMM -> DD + MM.MMMM/60
        int deg = (int)(lat / 100);
        float min = lat - (deg * 100);
        data.latitude = deg + (min / 60.0);
    }
    
    // N/S
    char* ns = ptr;
    ptr = getNextField(ptr);
    if (ns && strcmp(ns, "S") == 0) {
        data.latitude = -data.latitude;
    }
    
    // Longitude
    char* lonStr = ptr;
    ptr = getNextField(ptr);
    if (lonStr && strlen(lonStr) > 0) {
        float lon = atof(lonStr);
        int deg = (int)(lon / 100);
        float min = lon - (deg * 100);
        data.longitude = deg + (min / 60.0);
    }
    
    // E/W
    char* ew = ptr;
    ptr = getNextField(ptr);
    if (ew && strcmp(ew, "W") == 0) {
        data.longitude = -data.longitude;
    }
    
    // Fix Quality
    char* qualStr = ptr;
    ptr = getNextField(ptr);
    if (qualStr) {
        data.fixQuality = atoi(qualStr);
        data.isValid = (data.fixQuality > 0);
    }
    
    // Satellites
    char* satStr = ptr;
    ptr = getNextField(ptr);
    if (satStr) {
        data.satellites = atoi(satStr);
    }
    
    // Altitude
    ptr = getNextField(ptr); // HDOP
    char* altStr = ptr;
    ptr = getNextField(ptr);
    if (altStr) {
        data.altitude = atof(altStr);
    }
    
    data.lastUpdate = millis();
}

void GNSSModule::parseRMC(const char* sentence) {
    // $GNRMC или $GPRMC
    // Поля: Time, Status, Lat, N/S, Lon, E/W, Speed(knots), Course, Date, MagVar, E/W, Mode, Checksum
    
    char buffer[120];
    strncpy(buffer, sentence, sizeof(buffer));
    buffer[sizeof(buffer) - 1] = '\0';
    
    char* ptr = buffer;
    ptr = getNextField(ptr); // Time
    
    // Status (A=Active, V=Void)
    char* status = ptr;
    ptr = getNextField(ptr);
    if (status && strcmp(status, "A") != 0) {
        data.isValid = false;
        return;
    }
    
    // Skip Lat/Lon (уже есть в GGA)
    ptr = getNextField(ptr); // Lat
    ptr = getNextField(ptr); // N/S
    ptr = getNextField(ptr); // Lon
    ptr = getNextField(ptr); // E/W
    
    // Speed in knots
    char* speedStr = ptr;
    ptr = getNextField(ptr);
    if (speedStr) {
        float knots = atof(speedStr);
        data.speed = knots * 1.852; // Конвертация в км/ч
    }
    
    // Course
    char* courseStr = ptr;
    ptr = getNextField(ptr);
    if (courseStr) {
        data.course = atof(courseStr);
    }
    
    data.lastUpdate = millis();
}

void GNSSModule::parseVTG(const char* sentence) {
    // $GNVTG или $GPVTG
    // Поля: CourseTrue, T, CourseMag, M, SpeedKnots, N, SpeedKMH, K, Mode, Checksum
    
    char buffer[120];
    strncpy(buffer, sentence, sizeof(buffer));
    buffer[sizeof(buffer) - 1] = '\0';
    
    char* ptr = buffer;
    
    // Course True
    char* courseStr = ptr;
    ptr = getNextField(ptr);
    ptr = getNextField(ptr); // 'T'
    if (courseStr) {
        data.course = atof(courseStr);
    }
    
    // Skip Mag
    ptr = getNextField(ptr); 
    ptr = getNextField(ptr); 
    
    // Skip Knots
    ptr = getNextField(ptr);
    ptr = getNextField(ptr);
    
    // Speed KMH
    char* speedStr = ptr;
    ptr = getNextField(ptr);
    if (speedStr) {
        data.speed = atof(speedStr);
    }
    
    data.lastUpdate = millis();
}

void GNSSModule::update() {
    while (gpsSerial->available()) {
        char c = gpsSerial->read();
        
        // Простой парсер: ищем начало предложения '$'
        static char buffer[120];
        static int index = 0;
        
        if (c == '$') {
            index = 0;
        }
        
        if (index < sizeof(buffer) - 1) {
            buffer[index++] = c;
        }
        
        if (c == '\n') {
            buffer[index] = '\0';
            index = 0;
            
            // Определяем тип предложения и парсим
            if (strstr(buffer, "GGA") || strstr(buffer, "GGA")) {
                parseGGA(buffer);
            } else if (strstr(buffer, "RMC") || strstr(buffer, "RMC")) {
                parseRMC(buffer);
            } else if (strstr(buffer, "VTG") || strstr(buffer, "VTG")) {
                parseVTG(buffer);
            }
        }
    }
}

GNSSData GNSSModule::getData() const {
    return data;
}

bool GNSSModule::hasFreshData(unsigned long maxAgeMs) const {
    if (!data.isValid) return false;
    return (millis() - data.lastUpdate) < maxAgeMs;
}

float GNSSModule::distanceTo(float targetLat, float targetLon) const {
    if (!data.isValid) return -1.0;
    
    // Формула гаверсинусов для расчета расстояния
    const float R = 6371000.0; // Радиус Земли в метрах
    float dLat = radians(targetLat - data.latitude);
    float dLon = radians(targetLon - data.longitude);
    
    float a = sin(dLat / 2) * sin(dLat / 2) +
              cos(radians(data.latitude)) * cos(radians(targetLat)) *
              sin(dLon / 2) * sin(dLon / 2);
              
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    
    return R * c;
}

float GNSSModule::bearingTo(float targetLat, float targetLon) const {
    if (!data.isValid) return 0.0;
    
    float dLon = radians(targetLon - data.longitude);
    float lat1 = radians(data.latitude);
    float lat2 = radians(targetLat);
    
    float y = sin(dLon) * cos(lat2);
    float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    
    float bearing = degrees(atan2(y, x));
    return fmod(bearing + 360.0, 360.0);
}