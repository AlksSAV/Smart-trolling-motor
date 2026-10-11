#include <Arduino.h>
#include <Wire.h>

#define I2C_SDA 12
#define I2C_SCL 13

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- I2C Scanner для BNO085 ---");
  
  // Инициализация I2C на заданных пинах
  Wire.begin(I2C_SDA, I2C_SCL);
}

void loop() {
  byte error, address;
  int nDevices = 0;
  
  Serial.println("Сканирование шины I2C...");
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("Устройство найдено по адресу: 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    }
  }
  
  if (nDevices == 0) {
    Serial.println("Устройства не найдены! Проверь:");
    Serial.println("1. Подключение проводов (SDA/SCL не перепутаны?)");
    Serial.println("2. Питание 3.3В");
    Serial.println("3. Наличие подтягивающих резисторов на модуле");
  } else {
    Serial.println("Сканирование завершено.\n");
  }
  
  delay(3000);
}