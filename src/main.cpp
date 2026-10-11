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
#define GNSS_RX 18
#define GNSS_TX 17
HardwareSerial gnssSerial(1);
TinyGPSPlus gps;

const unsigned long GNSS_BAUD = 38400;
bool ubx_configured = false;
uint32_t parsed_sentences = 0;

void sendUBX(uint8_t cls, uint8_t id, uint8_t* payload, uint16_t length) {
  gnssSerial.write(0xB5);
  gnssSerial.write(0x62);
  gnssSerial.write(cls);
  gnssSerial.write(id);
  gnssSerial.write(length & 0xFF);
  gnssSerial.write((length >> 8) & 0xFF);
  
  uint8_t ck_a = 0, ck_b = 0;
  ck_a += cls; ck_b += ck_a;
  ck_a += id; ck_b += ck_a;
  ck_a += (length & 0xFF); ck_b += ck_a;
  ck_a += ((length >> 8) & 0xFF); ck_b += ck_a;
  
  for (uint16_t i = 0; i < length; i++) {
    gnssSerial.write(payload[i]);
    ck_a += payload[i];
    ck_b += ck_a;
  }
  
  gnssSerial.write(ck_a);
  gnssSerial.write(ck_b);
  delay(100);
}

void displaySystemStatus();

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  
  Serial.println("\n=== Smart Trolling Motor: IMU + GNSS ===");
  Serial.println("Waiting 2 seconds for modules to power up...");
  delay(2000);

  // 1. Init IMU
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  int imu_attempts = 0;
  while (!imu_ready && imu_attempts < 3) {
    imu_attempts++;
    if (bno08x.begin_I2C(BNO08X_ADDRESS)) {
      bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 50000);
      imu_ready = true;
    } else {
      delay(1000);
    }
  }

  // 2. Init GNSS
  gnssSerial.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX, GNSS_TX);
  
  // 3. Send UBX Config
  if (imu_ready) {
    Serial.println("[GNSS] Sending UBX config: 5Hz + Sea mode...");
    delay(500);

    // UBX-CFG-RATE: 200ms (5Hz)
    uint8_t ratePayload[] = {0xC8, 0x00, 0x01, 0x00, 0x00, 0x00};
    sendUBX(0x06, 0x08, ratePayload, sizeof(ratePayload));

    // UBX-CFG-NAV5: dynModel = 6 (Sea)
    uint8_t nav5Payload[36] = {0};
    nav5Payload[0] = 0x01; // mask
    nav5Payload[1] = 0x00; 
    nav5Payload[2] = 0x06; // Sea
    sendUBX(0x06, 0x24, nav5Payload, sizeof(nav5Payload));

    ubx_configured = true;
    Serial.println("[GNSS] Configuration applied successfully.\n");
  }
}

void loop() {
  if (imu_ready && bno08x.getSensorEvent(&sensorValue)) { }

  // Считаем, сколько полных NMEA-фраз успешно распарсилось
  while (gnssSerial.available() > 0) {
    if (gps.encode(gnssSerial.read())) {
      parsed_sentences++;
    }
  }

  static unsigned long last_print = 0;
  if (millis() - last_print > 2000) {
    displaySystemStatus();
    last_print = millis();
  }
}

void displaySystemStatus() {
  Serial.println("========================================");
  
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

  // Вычисляем реальную частоту получения данных (фраз в секунду)
  float hz = (parsed_sentences * 1000.0) / 2000.0; 
  parsed_sentences = 0; // Сброс счетчика

  Serial.printf("[GNSS] Rate: ~%.1f Hz | Sats: %2d | Fix: %s\n", 
                hz, gps.satellites.value(), gps.location.isValid() ? "YES" : "NO");
                
  if (ubx_configured) {
    Serial.println("[GNSS] Config: 5Hz + Sea Mode (ACTIVE)");
  }
  
  Serial.println("========================================\n");
}