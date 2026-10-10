#include "imu_module.h"
#include <math.h>

// Примечание: Для полноценной работы BNO085 рекомендуется использовать
// библиотеку Adafruit_BNO085 или SparkFun_BNO085_Arduino_Library.
// Данная реализация является базовым каркасом (stub).
// В platformio.ini добавьте: lib_deps = adafruit/Adafruit BNO085 @ ^1.2.0

IMUModule::IMUModule(TwoWire& wire, uint8_t addr) 
    : i2cBus(&wire), i2cAddr(addr) {
    data.isValid = false;
    data.heading = 0.0;
    data.pitch = 0.0;
    data.roll = 0.0;
    data.yaw = 0.0;
    data.accuracy = 0;
    data.lastUpdate = 0;
}

bool IMUModule::initBNO085() {
    // Проверка наличия устройства на шине I2C
    i2cBus->beginTransmission(i2cAddr);
    uint8_t error = i2cBus->endTransmission();
    
    if (error != 0) {
        Serial.printf("[IMU] BNO085 not found at address 0x%02X (error: %d)\n", i2cAddr, error);
        return false;
    }
    
    Serial.printf("[IMU] BNO085 found at address 0x%02X\n", i2cAddr);
    
    // TODO: Здесь должна быть инициализация через библиотеку
    // Например:
    // bno08x.begin_I2C(i2cAddr, i2cBus);
    // bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 50); // 50ms = 20Hz
    
    return true;
}

bool IMUModule::readSensorData() {
    // TODO: Заменить на реальное чтение через библиотеку
    // Пример с Adafruit_BNO085:
    /*
    if (bno08x.wasReset()) {
        Serial.println("[IMU] Sensor was reset");
        // Переинициализация репортов
    }
    
    if (!bno08x.getSensorEvent(&sensorValue)) {
        return false;
    }
    
    switch (sensorValue.sensorId) {
        case SH2_GAME_ROTATION_VECTOR:
            // Конвертация кватерниона в углы Эйлера
            float q[4] = {
                sensorValue.un.gameRotationVector.real,
                sensorValue.un.gameRotationVector.i,
                sensorValue.un.gameRotationVector.j,
                sensorValue.un.gameRotationVector.k
            };
            
            // Yaw (heading)
            float siny_cosp = 2.0 * (q[0] * q[3] + q[1] * q[2]);
            float cosy_cosp = 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]);
            data.yaw = atan2(siny_cosp, cosy_cosp) * 180.0 / PI;
            if (data.yaw < 0) data.yaw += 360.0;
            data.heading = data.yaw;
            
            // Pitch
            float sinp = 2.0 * (q[0] * q[2] - q[3] * q[1]);
            data.pitch = asin(sinp) * 180.0 / PI;
            
            // Roll
            float sinr_cosp = 2.0 * (q[0] * q[1] + q[2] * q[3]);
            float cosr_cosp = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
            data.roll = atan2(sinr_cosp, cosr_cosp) * 180.0 / PI;
            
            data.accuracy = sensorValue.status;
            data.isValid = true;
            data.lastUpdate = millis();
            return true;
    }
    */
    
    // STUB: Возвращаем заглушку для компиляции
    // Удалить этот блок после подключения реальной библиотеки!
    static unsigned long lastStubUpdate = 0;
    if (millis() - lastStubUpdate > 100) {
        data.heading = 0.0;  // Заглушка: всегда север
        data.pitch = 0.0;
        data.roll = 0.0;
        data.yaw = 0.0;
        data.accuracy = 3;   // Максимальная точность (заглушка)
        data.isValid = true;
        data.lastUpdate = millis();
        lastStubUpdate = millis();
        return true;
    }
    
    return false;
}

void IMUModule::begin() {
    // Инициализация I2C
    i2cBus->begin(PIN_IMU_SDA, PIN_IMU_SCL, 400000); // 400 kHz Fast Mode
    
    delay(100); // Задержка для стабилизации
    
    if (!initBNO085()) {
        Serial.println("[IMU] Initialization FAILED!");
        data.isValid = false;
        return;
    }
    
    Serial.println("[IMU] Initialized successfully");
}

void IMUModule::update() {
    readSensorData();
}

IMUData IMUModule::getData() const {
    return data;
}

bool IMUModule::hasFreshData(unsigned long maxAgeMs) const {
    if (!data.isValid) return false;
    return (millis() - data.lastUpdate) < maxAgeMs;
}

float IMUModule::getHeading() const {
    return data.heading;
}