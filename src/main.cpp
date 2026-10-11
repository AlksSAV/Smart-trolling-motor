#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <TinyGPSPlus.h>

// --- Hardware Pins ---
#define I2C_SDA 12
#define I2C_SCL 13
#define BNO08X_ADDRESS 0x4B
#define BNO08X_RESET -1

#define GNSS_RX 18
#define GNSS_TX 17

// --- Global Objects ---
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
bool imu_ready = false;

HardwareSerial gnssSerial(1);
TinyGPSPlus gps;
const unsigned long GNSS_BAUD = 38400;
bool ubx_configured = false;
uint32_t parsed_sentences = 0;

// --- State Machine ---
enum SystemMode {
    MODE_MANUAL,
    MODE_HEADING_HOLD,
    MODE_ANCHOR_HOLD
};

SystemMode currentMode = MODE_MANUAL;

// --- Target States ---
float targetYaw = 0.0;
double targetLat = 0.0;
double targetLon = 0.0;

// --- UBX Helper Function ---
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

// --- Math Helpers ---
float calculateBearing(double lat1, double lon1, double lat2, double lon2) {
    double dLon = (lon2 - lon1) * PI / 180.0;
    double rLat1 = lat1 * PI / 180.0;
    double rLat2 = lat2 * PI / 180.0;
    double y = sin(dLon) * cos(rLat2);
    double x = cos(rLat1) * sin(rLat2) - sin(rLat1) * cos(rLat2) * cos(dLon);
    float bearing = atan2(y, x) * 180.0 / PI;
    return fmod((bearing + 360.0), 360.0);
}

float normalizeAngleDiff(float target, float current) {
    float diff = target - current;
    while (diff > 180.0) diff -= 360.0;
    while (diff < -180.0) diff += 360.0;
    return diff;
}

void displaySystemStatus();
void processSerialCommands();

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    
    Serial.println("\n=== Smart Trolling Motor: State Machine & Bearing Test ===");
    Serial.println("Waiting 2 seconds for modules to power up...");
    delay(2000);

    // 1. Init IMU with verbose logging
    Serial.println("\n[IMU] Initializing BNO085...");
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);
    
    int imu_attempts = 0;
    while (!imu_ready && imu_attempts < 3) {
        imu_attempts++;
        Serial.printf("[IMU] Attempt %d/3...\n", imu_attempts);
        
        if (bno08x.begin_I2C(BNO08X_ADDRESS)) {
            Serial.println("[IMU] ✓ SUCCESS: BNO085 initialized at address 0x4B");
            bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 50000);
            imu_ready = true;
        } else {
            Serial.println("[IMU] ✗ Failed. Retrying in 1 second...");
            delay(1000);
        }
    }
    
    if (!imu_ready) {
        Serial.println("\n[IMU] ✗✗✗ CRITICAL ERROR: BNO085 not responding!");
        Serial.println("[IMU] Check: Power (3.3V), I2C wiring (SDA=12, SCL=13)");
    }

    // 2. Init GNSS
    Serial.println("\n[GNSS] Initializing UART1...");
    gnssSerial.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX, GNSS_TX);
    Serial.printf("[GNSS] ✓ Listening at %lu baud.\n", GNSS_BAUD);
    
    // 3. Send UBX Config (only if IMU is ready to avoid power issues)
    if (imu_ready) {
        Serial.println("[GNSS] Sending UBX config: 5Hz + Sea mode...");
        delay(500);
        uint8_t ratePayload[] = {0xC8, 0x00, 0x01, 0x00, 0x00, 0x00};
        sendUBX(0x06, 0x08, ratePayload, sizeof(ratePayload));
        uint8_t nav5Payload[36] = {0};
        nav5Payload[0] = 0x01; nav5Payload[1] = 0x00; nav5Payload[2] = 0x06;
        sendUBX(0x06, 0x24, nav5Payload, sizeof(nav5Payload));
        ubx_configured = true;
        Serial.println("[GNSS] ✓ Configuration applied.\n");
    }

    Serial.println(">>> Serial Commands: 'M' = Manual, 'H' = Heading Hold, 'A' = Anchor Hold <<<\n");
    Serial.println("💡 TIP: If you don't see this setup log, press the RESET button on your ESP32 board!");
}

void loop() {
    // 1. Read IMU (only if ready)
    if (imu_ready) {
        bno08x.getSensorEvent(&sensorValue);
    }

    // 2. Read GNSS
    while (gnssSerial.available() > 0) {
        if (gps.encode(gnssSerial.read())) {
            parsed_sentences++;
        }
    }

    // 3. Process User Commands
    processSerialCommands();

    // 4. Periodic Status Update
    static unsigned long last_print = 0;
    if (millis() - last_print > 1000) {
        displaySystemStatus();
        last_print = millis();
    }
}

void processSerialCommands() {
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        cmd = toupper(cmd);
        
        if (cmd == 'M') {
            currentMode = MODE_MANUAL;
            Serial.println("\n[MODE] Switched to MANUAL");
        } 
        else if (cmd == 'H') {
            if (!imu_ready) {
                Serial.println("\n[ERROR] Cannot set Heading Hold: IMU not ready!");
                return;
            }
            currentMode = MODE_HEADING_HOLD;
            float real = sensorValue.un.arvrStabilizedRV.real;
            float i = sensorValue.un.arvrStabilizedRV.i;
            float j = sensorValue.un.arvrStabilizedRV.j;
            float k = sensorValue.un.arvrStabilizedRV.k;
            targetYaw = atan2(2.0 * (real * k + i * j), 1.0 - 2.0 * (j * j + k * k)) * 180.0 / PI;
            if (targetYaw < 0) targetYaw += 360.0;
            Serial.printf("\n[MODE] Switched to HEADING HOLD. Target Yaw locked at: %.1f°\n", targetYaw);
        } 
        else if (cmd == 'A') {
            if (!gps.location.isValid()) {
                Serial.println("\n[ERROR] Cannot lock Anchor: GNSS fix not valid yet!");
                return;
            }
            currentMode = MODE_ANCHOR_HOLD;
            targetLat = gps.location.lat();
            targetLon = gps.location.lng();
            Serial.printf("\n[MODE] Switched to ANCHOR HOLD. Target locked at: Lat %.6f, Lon %.6f\n", targetLat, targetLon);
        }
    }
}

void displaySystemStatus() {
    Serial.println("----------------------------------------");
    
    // Mode
    String modeStr = (currentMode == MODE_MANUAL) ? "MANUAL" : 
                     (currentMode == MODE_HEADING_HOLD) ? "HEADING HOLD" : "ANCHOR HOLD";
    Serial.printf("[SYS] Mode: %-12s\n", modeStr.c_str());

    // IMU Status with protection
    if (imu_ready) {
        float real = sensorValue.un.arvrStabilizedRV.real;
        float i = sensorValue.un.arvrStabilizedRV.i;
        float j = sensorValue.un.arvrStabilizedRV.j;
        float k = sensorValue.un.arvrStabilizedRV.k;
        float currentYaw = atan2(2.0 * (real * k + i * j), 1.0 - 2.0 * (j * j + k * k)) * 180.0 / PI;
        if (currentYaw < 0) currentYaw += 360.0;
        Serial.printf("[IMU] ✓ Yaw: %6.1f°\n", currentYaw);
    } else {
        Serial.println("[IMU] ✗ NOT READY (check setup log)");
    }

    // GNSS Stats
    float hz = (parsed_sentences * 1000.0) / 1000.0; 
    parsed_sentences = 0;
    Serial.printf("[GNSS] Rate: ~%.0f Hz | Sats: %2d | Fix: %s\n", 
                  hz, gps.satellites.value(), gps.location.isValid() ? "YES" : "NO");

    // Logic & Error Calculation
    if (currentMode == MODE_HEADING_HOLD && imu_ready) {
        float real = sensorValue.un.arvrStabilizedRV.real;
        float i = sensorValue.un.arvrStabilizedRV.i;
        float j = sensorValue.un.arvrStabilizedRV.j;
        float k = sensorValue.un.arvrStabilizedRV.k;
        float currentYaw = atan2(2.0 * (real * k + i * j), 1.0 - 2.0 * (j * j + k * k)) * 180.0 / PI;
        if (currentYaw < 0) currentYaw += 360.0;
        
        float yawError = normalizeAngleDiff(targetYaw, currentYaw);
        Serial.printf("[PID] Target: %6.1f° | Error: %6.1f° (Steer: %s)\n", 
                      targetYaw, yawError, yawError > 0 ? "LEFT" : "RIGHT");
    } 
    else if (currentMode == MODE_ANCHOR_HOLD && gps.location.isValid()) {
        float real = sensorValue.un.arvrStabilizedRV.real;
        float i = sensorValue.un.arvrStabilizedRV.i;
        float j = sensorValue.un.arvrStabilizedRV.j;
        float k = sensorValue.un.arvrStabilizedRV.k;
        float currentYaw = atan2(2.0 * (real * k + i * j), 1.0 - 2.0 * (j * j + k * k)) * 180.0 / PI;
        if (currentYaw < 0) currentYaw += 360.0;
        
        float bearingToTarget = calculateBearing(
            gps.location.lat(), gps.location.lng(), 
            targetLat, targetLon
        );
        float distanceMeters = gps.distanceBetween(
            gps.location.lat(), gps.location.lng(), 
            targetLat, targetLon
        );
        float headingError = normalizeAngleDiff(bearingToTarget, currentYaw);
        
        Serial.printf("[PID] Dist: %5.1fm | Bearing: %6.1f° | Head Error: %6.1f°\n", 
                      distanceMeters, bearingToTarget, headingError);
    }
    
    Serial.println("----------------------------------------\n");
}