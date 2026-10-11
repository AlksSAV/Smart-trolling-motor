#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <TinyGPSPlus.h>

// --- IMU BNO085 ---
#define I2C_SDA 12
#define I2C_SCL 13
#define BNO08X_ADDRESS 0x4B
#define BNO08X_RESET -1

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
bool imu_ready = false;

// --- GNSS NEO-M10N ---
#define GNSS_RX 18  // ESP32 RX <- GNSS TX
#define GNSS_TX 17  // ESP32 TX -> GNSS RX
HardwareSerial gnssSerial(1); // Use UART1
TinyGPSPlus gps;

// Подтвержденная скорость для этого модуля
const unsigned long GNSS_BAUD = 38400;

void displaySystemStatus();

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  
  Serial.println("\n=== Smart Trolling Motor: IMU + GNSS Unified ===");

  // 1. Init IMU (I2C)
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); // 400 kHz
  if (!bno08x.begin_I2C(BNO08X_ADDRESS)) {
    Serial.println("[IMU] ERROR: BNO085 not found at 0x4B!");
  } else {
    Serial.println("[IMU] OK: BNO085 initialized.");
    bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 50000); // 20Hz
    imu_ready = true;
  }

  // 2. Init GNSS (UART1)
  gnssSerial.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX, GNSS_TX);
  Serial.printf("[GNSS] OK: Listening on UART1 (RX:18, TX:17) at %lu baud.\n", GNSS_BAUD);
  Serial.println("[GNSS] Waiting for NMEA data (indoors: fix will be 0, this is normal)...\n");
}

void loop() {
  // --- IMU Loop ---
  if (imu_ready && bno08x.getSensorEvent(&sensorValue)) {
    // Данные обновляются в фоновой структуре sensorValue
  }

  // --- GNSS Loop ---
  // Читаем все доступные байты из UART1 и скармливаем парсеру
  while (gnssSerial.available() > 0) {
    gps.encode(gnssSerial.read());
  }

  // Выводим сводный статус каждые 2 секунды
  static unsigned long last_print = 0;
  if (millis() - last_print > 2000) {
    displaySystemStatus();
    last_print = millis();
  }
}

void displaySystemStatus() {
  Serial.println("========================================");
  
  // IMU Data
  if (imu_ready) {
    float real = sensorValue.un.arvrStabilizedRV.real;
    float i = sensorValue.un.arvrStabilizedRV.i;
    float j = sensorValue.un.arvrStabilizedRV.j;
    float k = sensorValue.un.arvrStabilizedRV.k;
    
    float yaw = atan2(2.0 * (real * k + i * j), 1.0 - 2.0 * (j * j + k * k)) * 180.0 / PI;
    if (yaw < 0) yaw += 360.0;
    
    Serial.printf("[IMU] Yaw: %6.1f°\n", yaw);
  } else {
    Serial.println("[IMU] Not ready");
  }

  // GNSS Data
  Serial.printf("[GNSS] Sats: %2d | Fix: %s\n", 
                gps.satellites.value(), 
                gps.location.isValid() ? "YES" : "NO (Indoors)");
                
  if (gps.location.isValid()) {
    Serial.printf("[GNSS] Lat: %.6f, Lng: %.6f\n", gps.location.lat(), gps.location.lng());
  }
  
  if (gps.speed.isValid()) {
    Serial.printf("[GNSS] Speed: %.2f km/h\n", gps.speed.kmph());
  }
  
  Serial.println("========================================\n");
}