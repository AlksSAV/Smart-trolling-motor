// Smart Trolling Motor - main entry point
// Объединяет все модули: ESC, Servo, RC, GNSS, IMU, Position Hold

#include <Arduino.h>
#include "config.h"
#include "esc_control.h"
#include "servo_control.h"
#include "rc_receiver.h"
#include "gnss_module.h"
#include "imu_module.h"
#include "position_hold.h"

// Создаём экземпляры модулей
ESCControl esc(PIN_ESC_PWM);
ServoControl servo(PIN_SERVO_PWM);
RCReceiver rcReceiver;
GNSSModule gnss(Serial1);
IMUModule imu(Wire, IMU_I2C_ADDR);

// PositionHold будет создан после инициализации всех модулей
PositionHold* positionHold = nullptr;
void printDebugInfo();  // Добавьте эту строку перед void setup()
void setup() {
    // Инициализация Serial для отладки (USB CDC)
    Serial.begin(115200);
    delay(2000); // Задержка для открытия монитора порта
    
    Serial.println("\n========================================");
    Serial.println("  Smart Trolling Motor v1.0");
    Serial.println("========================================\n");
    
    // Информация о плате
    Serial.printf("Chip:      %s\n", ESP.getChipModel());
    Serial.printf("Flash:     %lu MB\n", ESP.getFlashChipSize() / 1048576UL);
    if (psramFound()) {
        Serial.printf("PSRAM:     found, size %lu bytes, free %lu bytes\n",
                      ESP.getPsramSize(), ESP.getFreePsram());
    } else {
        Serial.println("PSRAM:     NOT FOUND");
    }
    Serial.printf("CPU Freq:  %lu MHz\n", ESP.getCpuFreqMHz());
    Serial.println();
    
    // Инициализация модулей
    Serial.println("[INIT] Initializing modules...");
    
    // ESC
    esc.begin();
    Serial.println("  ✓ ESC initialized");
    
    // Servo
    servo.begin();
    Serial.println("  ✓ Servo initialized");
    
    // RC Receiver
    rcReceiver.begin();
    Serial.println("  ✓ RC Receiver initialized");
    
    // GNSS
    gnss.begin();
    Serial.println("  ✓ GNSS module initialized");
    
    // IMU
    imu.begin();
    Serial.println("  ✓ IMU module initialized");
    
    // Position Hold
    positionHold = new PositionHold(gnss, imu, esc, servo, rcReceiver);
    positionHold->begin();
    Serial.println("  ✓ Position Hold initialized");
    
    Serial.println("\n[READY] System ready!\n");
}

void loop() {
    // Обновление всех модулей
    rcReceiver.update();
    gnss.update();
    imu.update();
    
    // Обновление режима удержания позиции
    if (positionHold) {
        positionHold->update();
    }
    
    // Отладочная информация каждые 2 секунды
    static unsigned long lastDebugPrint = 0;
    if (millis() - lastDebugPrint > 2000) {
        printDebugInfo();
        lastDebugPrint = millis();
    }
    
    // Небольшая задержка для стабильности
    delay(10);
}

void printDebugInfo() {
    Serial.println("--- Debug Info ---");
    
    // RC каналы
    RCChannels rc = rcReceiver.getChannels();
    Serial.printf("RC: CH1=%d CH2=%d CH3=%d CH4=%d CH5=%d CH6=%d\n",
                  rc.ch1, rc.ch2, rc.ch3, rc.ch4, rc.ch5, rc.ch6);
    
    // GNSS данные
    GNSSData gps = gnss.getData();
    if (gps.isValid) {
        Serial.printf("GNSS: Lat=%.6f Lon=%.6f Alt=%.1fm Sat=%d Speed=%.1f km/h Course=%.1f°\n",
                      gps.latitude, gps.longitude, gps.altitude, 
                      gps.satellites, gps.speed, gps.course);
    } else {
        Serial.println("GNSS: No valid fix");
    }
    
    // IMU данные
    IMUData imuData = imu.getData();
    if (imuData.isValid) {
        Serial.printf("IMU: Heading=%.1f° Pitch=%.1f° Roll=%.1f° Acc=%d\n",
                      imuData.heading, imuData.pitch, imuData.roll, imuData.accuracy);
    } else {
        Serial.println("IMU: No data");
    }
    
    // ESC и Servo
    Serial.printf("ESC: Throttle=%d%% | Servo: Angle=%d°\n",
                  esc.getThrottle(), servo.getAngle());
    
    Serial.println("------------------\n");
}