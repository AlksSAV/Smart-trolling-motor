#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

#define I2C_SDA 12
#define I2C_SCL 13
#define BNO08X_ADDRESS 0x4B  // Китайский модуль использует 0x4B
#define BNO08X_RESET -1      // Пин RST не используем

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

void setReports(sh2_SensorId_t sensorType, uint32_t reportIntervalUs) {
  if (!bno08x.enableReport(sensorType, reportIntervalUs)) {
    Serial.printf("Ошибка включения сенсора 0x%X!\n", sensorType);
  }
}

void setup(void) {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  
  Serial.println("\n=== Инициализация BNO085 (адрес 0x4B) ===");
  
  // Инициализация I2C с кастомными пинами
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); // 400 кГц для стабильности BNO085
  
  // ВАЖНО: Явно указываем адрес 0x4B
  if (!bno08x.begin_I2C(BNO08X_ADDRESS)) {
    Serial.println("КРИТИЧЕСКАЯ ОШИБКА: Не удалось найти BNO08x по адресу 0x4B!");
    Serial.println("Проверь:");
    Serial.println("1. Питание модуля (3.3В)");
    Serial.println("2. Подключение SDA->GPIO12, SCL->GPIO13");
    Serial.println("3. Попробуй отключить питание модуля на 10 сек и включить снова");
    while (1) { delay(100); }
  }
  
  Serial.println("✓ BNO085 успешно инициализирован!");
  
  // Включаем "ARVR Stabilized Rotation Vector" с интервалом 50 мс (20 Гц)
  // Это самый стабильный режим для определения курса
  setReports(SH2_ARVR_STABILIZED_RV, 50000);
  
  Serial.println("✓ Режим Rotation Vector включен (20 Гц)");
  Serial.println("\nОжидаю данные датчика...\n");
}

void loop(void) {
  if (bno08x.getSensorEvent(&sensorValue)) {
    
    if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
      
      // Извлекаем кватернион
      float real = sensorValue.un.arvrStabilizedRV.real;
      float i = sensorValue.un.arvrStabilizedRV.i;
      float j = sensorValue.un.arvrStabilizedRV.j;
      float k = sensorValue.un.arvrStabilizedRV.k;
      
      // Вычисляем углы Эйлера (радианы -> градусы)
      float yaw = atan2(2.0 * (real * k + i * j), 1.0 - 2.0 * (j * j + k * k));
      float pitch = asin(2.0 * (real * j - k * i));
      float roll = atan2(2.0 * (real * i + j * k), 1.0 - 2.0 * (i * i + j * j));
      
      yaw = yaw * 180.0 / PI;
      pitch = pitch * 180.0 / PI;
      roll = roll * 180.0 / PI;
      
      // Нормализуем Yaw в диапазон 0-360°
      if (yaw < 0) yaw += 360.0;

      Serial.printf("Yaw: %6.1f° | Pitch: %6.1f° | Roll: %6.1f°\n", 
                    yaw, pitch, roll);
    }
  }
}