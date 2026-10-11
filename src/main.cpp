#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <TinyGPSPlus.h>

// ============================================================
// Hardware Pin Assignment (see docs/pinmap.md)
// ============================================================
#define I2C_SDA         12
#define I2C_SCL         13
#define BNO08X_ADDRESS  0x4B
#define BNO08X_RESET    -1

#define GNSS_RX         18   // ESP32 RX <- GNSS TX
#define GNSS_TX         17   // ESP32 TX -> GNSS RX

#define ESC_PIN         15   // LEDC ch0, 50Hz, 1000-2000us
#define SERVO_PIN       16   // LEDC ch1, 50Hz, +/-45 deg

// ============================================================
// Global Objects
// ============================================================
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
bool imu_ready = false;

HardwareSerial gnssSerial(1);
TinyGPSPlus gps;
const unsigned long GNSS_BAUD = 38400;
bool ubx_configured = false;
uint32_t parsed_sentences = 0;

// ============================================================
// State Machine
// ============================================================
enum SystemMode {
    MODE_MANUAL,
    MODE_HEADING_HOLD,
    MODE_ANCHOR_HOLD
};

SystemMode currentMode = MODE_MANUAL;

// ============================================================
// Target States
// ============================================================
float targetYaw = 0.0;
double targetLat = 0.0;
double targetLon = 0.0;
bool isSimulatedTarget = false;

// ============================================================
// ArduPilot-inspired Loiter Parameters
// ============================================================
const float LOITER_RADIUS_M    = 2.0;    // Dead zone radius (meters)
const float MAX_DISTANCE_M     = 10.0;   // Distance for 100% throttle
const float SERVO_P_GAIN       = 8.0;    // PWM us per degree of heading error
const float LOITER_SPEED_GAIN  = 0.15;   // Speed factor per meter outside radius

// PWM limits
const int ESC_NEUTRAL_US       = 1500;
const int ESC_MAX_FORWARD_US   = 1800;   // Max forward throttle
const int SERVO_NEUTRAL_US     = 1500;
const int SERVO_MIN_US         = 1000;   // -45 deg
const int SERVO_MAX_US         = 2000;   // +45 deg

// LEDC config (ESP32 Core v3.x)
const int LEDC_FREQ            = 50;
const int LEDC_RESOLUTION      = 14;     // 14-bit = 16384 steps

// ============================================================
// UBX Helper: send binary config to GNSS module
// ============================================================
void sendUBX(uint8_t cls, uint8_t id, uint8_t* payload, uint16_t length) {
    gnssSerial.write(0xB5);
    gnssSerial.write(0x62);
    gnssSerial.write(cls);
    gnssSerial.write(id);
    gnssSerial.write(length & 0xFF);
    gnssSerial.write((length >> 8) & 0xFF);

    uint8_t ck_a = 0, ck_b = 0;
    ck_a += cls; ck_b += ck_a;
    ck_a += id;  ck_b += ck_a;
    ck_a += (length & 0xFF);       ck_b += ck_a;
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

// ============================================================
// Math Helpers
// ============================================================
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
    while (diff > 180.0)  diff -= 360.0;
    while (diff < -180.0) diff += 360.0;
    return diff;
}

float constrainFloat(float val, float minVal, float maxVal) {
    if (val < minVal) return minVal;
    if (val > maxVal) return maxVal;
    return val;
}

// ============================================================
// LEDC Helper: convert microseconds to duty cycle
// ============================================================
uint32_t usToDuty(uint32_t us) {
    // 14-bit: 2^14 = 16384, period = 20000us at 50Hz
    return (us * 16384) / 20000;
}

// ============================================================
// Get current yaw from IMU (degrees, 0-360)
// ============================================================
float getCurrentYaw() {
    float real = sensorValue.un.arvrStabilizedRV.real;
    float i    = sensorValue.un.arvrStabilizedRV.i;
    float j    = sensorValue.un.arvrStabilizedRV.j;
    float k    = sensorValue.un.arvrStabilizedRV.k;
    float yaw  = atan2(2.0 * (real * k + i * j),
                       1.0 - 2.0 * (j * j + k * k)) * 180.0 / PI;
    if (yaw < 0) yaw += 360.0;
    return yaw;
}

// ============================================================
// Forward Declarations
// ============================================================
void processSerialCommands();
void calculateAndApplyControl();
void displaySystemStatus();

// ============================================================
// Setup
// ============================================================
void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }

    Serial.println("\n========================================");
    Serial.println("  Smart Trolling Motor v0.3.0");
    Serial.println("  IMU + GNSS + ArduPilot Loiter Logic");
    Serial.println("  Servo (GPIO16) + ESC (GPIO15)");
    Serial.println("========================================");
    Serial.println("Waiting 2 seconds for modules to power up...");
    delay(2000);

    // --- 1. Init IMU (I2C) ---
    Serial.println("\n[IMU] Initializing BNO085...");
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);

    int imu_attempts = 0;
    while (!imu_ready && imu_attempts < 3) {
        imu_attempts++;
        Serial.printf("[IMU] Attempt %d/3...\n", imu_attempts);
        if (bno08x.begin_I2C(BNO08X_ADDRESS)) {
            bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 50000); // 20Hz
            imu_ready = true;
            Serial.println("[IMU] OK: BNO085 initialized at 0x4B");
        } else {
            Serial.println("[IMU] Failed. Retrying...");
            delay(1000);
        }
    }
    if (!imu_ready) {
        Serial.println("[IMU] CRITICAL ERROR: BNO085 not responding!");
    }

    // --- 2. Init GNSS (UART1) ---
    Serial.println("\n[GNSS] Initializing UART1...");
    gnssSerial.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX, GNSS_TX);
    Serial.printf("[GNSS] OK: Listening at %lu baud.\n", GNSS_BAUD);

    // --- 3. Send UBX Config ---
    if (imu_ready) {
        Serial.println("[GNSS] Sending UBX config: 5Hz + Sea mode...");
        delay(500);

        // UBX-CFG-RATE: 200ms (5Hz)
        uint8_t ratePayload[] = {0xC8, 0x00, 0x01, 0x00, 0x00, 0x00};
        sendUBX(0x06, 0x08, ratePayload, sizeof(ratePayload));

        // UBX-CFG-NAV5: dynModel = 6 (Sea)
        uint8_t nav5Payload[36] = {0};
        nav5Payload[0] = 0x01;  // mask: apply dynModel
        nav5Payload[1] = 0x00;
        nav5Payload[2] = 0x06;  // Sea
        sendUBX(0x06, 0x24, nav5Payload, sizeof(nav5Payload));

        ubx_configured = true;
        Serial.println("[GNSS] OK: Configuration applied.");
    }

    // --- 4. Init LEDC for ESC and Servo ---
    Serial.println("\n[LEDC] Initializing PWM outputs...");

    // ESC on GPIO 15
    ledcAttach(ESC_PIN, LEDC_FREQ, LEDC_RESOLUTION);
    ledcWrite(ESC_PIN, usToDuty(ESC_NEUTRAL_US));
    Serial.printf("[LEDC] ESC  (GPIO %d): neutral %d us\n", ESC_PIN, ESC_NEUTRAL_US);

    // Servo on GPIO 16
    ledcAttach(SERVO_PIN, LEDC_FREQ, LEDC_RESOLUTION);
    ledcWrite(SERVO_PIN, usToDuty(SERVO_NEUTRAL_US));
    Serial.printf("[LEDC] Servo (GPIO %d): neutral %d us\n", SERVO_PIN, SERVO_NEUTRAL_US);

    // --- 5. Print help ---
    Serial.println("\n>>> Serial Commands:");
    Serial.println("    'M' = Manual (ESC neutral, servo neutral)");
    Serial.println("    'H' = Heading Hold (lock current yaw, servo active)");
    Serial.println("    'A' = Anchor Hold (lock GPS position, full loiter)");
    Serial.println("    'S' = Simulate Anchor (fake target 5m at 90 deg)");
    Serial.println("========================================\n");
}

// ============================================================
// Main Loop
// ============================================================
void loop() {
    // 1. Read IMU
    if (imu_ready) {
        bno08x.getSensorEvent(&sensorValue);
    }

    // 2. Read GNSS
    while (gnssSerial.available() > 0) {
        if (gps.encode(gnssSerial.read())) {
            parsed_sentences++;
        }
    }

    // 3. Process Serial Commands
    processSerialCommands();

    // 4. Calculate and apply control outputs (20 Hz)
    static unsigned long last_control = 0;
    if (millis() - last_control > 50) {
        calculateAndApplyControl();
        last_control = millis();
    }

    // 5. Display status (1 Hz)
    static unsigned long last_print = 0;
    if (millis() - last_print > 1000) {
        displaySystemStatus();
        last_print = millis();
    }
}

// ============================================================
// Serial Command Processing
// ============================================================
void processSerialCommands() {
    if (!Serial.available()) return;

    char cmd = toupper(Serial.read());

    if (cmd == 'M') {
        currentMode = MODE_MANUAL;
        isSimulatedTarget = false;
        // Reset outputs to neutral
        ledcWrite(ESC_PIN, usToDuty(ESC_NEUTRAL_US));
        ledcWrite(SERVO_PIN, usToDuty(SERVO_NEUTRAL_US));
        Serial.println("\n[MODE] MANUAL");
    }
    else if (cmd == 'H') {
        if (!imu_ready) {
            Serial.println("\n[ERROR] IMU not ready!");
            return;
        }
        currentMode = MODE_HEADING_HOLD;
        isSimulatedTarget = false;
        targetYaw = getCurrentYaw();
        Serial.printf("\n[MODE] HEADING HOLD. Target Yaw: %.1f deg\n", targetYaw);
    }
    else if (cmd == 'A') {
        if (!gps.location.isValid()) {
            Serial.println("\n[ERROR] No GNSS fix. Use 'S' to simulate indoors.");
            return;
        }
        currentMode = MODE_ANCHOR_HOLD;
        isSimulatedTarget = false;
        targetLat = gps.location.lat();
        targetLon = gps.location.lng();
        Serial.printf("\n[MODE] ANCHOR HOLD. Target: %.6f, %.6f\n", targetLat, targetLon);
    }
    else if (cmd == 'S') {
        currentMode = MODE_ANCHOR_HOLD;
        isSimulatedTarget = true;
        targetLat = 55.7558;  // Fake coordinates
        targetLon = 37.6173;
        Serial.println("\n[MODE] ANCHOR HOLD (SIMULATED). Target: 5m at 90 deg");
    }
}

// ============================================================
// ArduPilot-inspired Control Logic
// ============================================================
void calculateAndApplyControl() {
    if (!imu_ready) return;

    float currentYaw = getCurrentYaw();
    int escPWM = ESC_NEUTRAL_US;
    int servoPWM = SERVO_NEUTRAL_US;

    if (currentMode == MODE_MANUAL) {
        // Manual: outputs stay neutral (RC receiver will override later)
        escPWM = ESC_NEUTRAL_US;
        servoPWM = SERVO_NEUTRAL_US;
    }
    else if (currentMode == MODE_HEADING_HOLD) {
        // Heading Hold: only servo active, ESC neutral
        float yawError = normalizeAngleDiff(targetYaw, currentYaw);
        float servoDev = constrainFloat(yawError * SERVO_P_GAIN,
                                         -(SERVO_NEUTRAL_US - SERVO_MIN_US),
                                         (SERVO_MAX_US - SERVO_NEUTRAL_US));
        servoPWM = SERVO_NEUTRAL_US + (int)servoDev;
        escPWM = ESC_NEUTRAL_US;
    }
    else if (currentMode == MODE_ANCHOR_HOLD) {
        // Anchor Hold: full ArduPilot Loiter Logic
        float distance = 0.0;
        float bearing = 0.0;

        if (isSimulatedTarget) {
            distance = 5.0;
            bearing = 90.0;
        } else if (gps.location.isValid()) {
            distance = gps.distanceBetween(
                gps.location.lat(), gps.location.lng(),
                targetLat, targetLon);
            bearing = calculateBearing(
                gps.location.lat(), gps.location.lng(),
                targetLat, targetLon);
        }

        float headingError = normalizeAngleDiff(bearing, currentYaw);

        // ArduPilot Yaw Error Ratio: reduce speed when heading error is large
        // 45 deg error -> 75% speed, 90 deg error -> 50% speed
        float yawErrorRatio = 1.0 - (constrainFloat(fabsf(headingError), 0.0, 90.0) / 90.0) * 0.5;

        if (distance > LOITER_RADIUS_M) {
            // Outside loiter radius: P-controller for speed
            float distanceError = distance - LOITER_RADIUS_M;
            float speedFactor = constrainFloat(
                distanceError / (MAX_DISTANCE_M - LOITER_RADIUS_M), 0.0, 1.0);
            float effectiveSpeed = speedFactor * yawErrorRatio;

            escPWM = ESC_NEUTRAL_US + (int)(effectiveSpeed * (ESC_MAX_FORWARD_US - ESC_NEUTRAL_US));

            // Servo: steer towards target
            float servoDev = constrainFloat(headingError * SERVO_P_GAIN,
                                             -(SERVO_NEUTRAL_US - SERVO_MIN_US),
                                             (SERVO_MAX_US - SERVO_NEUTRAL_US));
            servoPWM = SERVO_NEUTRAL_US + (int)servoDev;
        } else {
            // Inside loiter radius: stop motor, maintain heading
            escPWM = ESC_NEUTRAL_US;
            float servoDev = constrainFloat(headingError * SERVO_P_GAIN,
                                             -(SERVO_NEUTRAL_US - SERVO_MIN_US),
                                             (SERVO_MAX_US - SERVO_NEUTRAL_US));
            servoPWM = SERVO_NEUTRAL_US + (int)servoDev;
        }
    }

    // Apply PWM outputs
    ledcWrite(ESC_PIN, usToDuty(escPWM));
    ledcWrite(SERVO_PIN, usToDuty(servoPWM));
}

// ============================================================
// Status Display
// ============================================================
void displaySystemStatus() {
    Serial.println("----------------------------------------");

    // Mode
    String modeStr;
    switch (currentMode) {
        case MODE_MANUAL:       modeStr = "MANUAL";       break;
        case MODE_HEADING_HOLD: modeStr = "HEADING HOLD"; break;
        case MODE_ANCHOR_HOLD:  modeStr = "ANCHOR HOLD";  break;
    }
    if (isSimulatedTarget) modeStr += " (SIM)";
    Serial.printf("[SYS] Mode: %-16s\n", modeStr.c_str());

    // IMU
    float currentYaw = 0.0;
    if (imu_ready) {
        currentYaw = getCurrentYaw();
        Serial.printf("[IMU] Yaw: %6.1f deg\n", currentYaw);
    } else {
        Serial.println("[IMU] NOT READY");
    }

    // GNSS
    float hz = (parsed_sentences * 1000.0) / 1000.0;
    parsed_sentences = 0;
    Serial.printf("[GNSS] ~%.0f Hz | Sats: %2d | Fix: %s\n",
                  hz, gps.satellites.value(),
                  gps.location.isValid() ? "YES" : "NO");

    // Control outputs (read back what was applied)
    if (currentMode == MODE_HEADING_HOLD && imu_ready) {
        float yawError = normalizeAngleDiff(targetYaw, currentYaw);
        float servoDev = constrainFloat(yawError * SERVO_P_GAIN,
                                         -(SERVO_NEUTRAL_US - SERVO_MIN_US),
                                         (SERVO_MAX_US - SERVO_NEUTRAL_US));
        int servoPWM = SERVO_NEUTRAL_US + (int)servoDev;
        Serial.printf("[OUT] Target: %6.1f deg | Err: %6.1f deg | Servo: %4d us\n",
                      targetYaw, yawError, servoPWM);
    }
    else if (currentMode == MODE_ANCHOR_HOLD && imu_ready) {
        float distance = isSimulatedTarget ? 5.0 :
            (gps.location.isValid() ?
                gps.distanceBetween(gps.location.lat(), gps.location.lng(), targetLat, targetLon) : 0.0);
        float bearing = isSimulatedTarget ? 90.0 :
            (gps.location.isValid() ?
                calculateBearing(gps.location.lat(), gps.location.lng(), targetLat, targetLon) : 0.0);
        float headingError = normalizeAngleDiff(bearing, currentYaw);
        float yawErrorRatio = 1.0 - (constrainFloat(fabsf(headingError), 0.0, 90.0) / 90.0) * 0.5;

        int escPWM = ESC_NEUTRAL_US;
        int servoPWM = SERVO_NEUTRAL_US;

        if (distance > LOITER_RADIUS_M) {
            float speedFactor = constrainFloat(
                (distance - LOITER_RADIUS_M) / (MAX_DISTANCE_M - LOITER_RADIUS_M), 0.0, 1.0);
            escPWM = ESC_NEUTRAL_US + (int)(speedFactor * yawErrorRatio * (ESC_MAX_FORWARD_US - ESC_NEUTRAL_US));
            float servoDev = constrainFloat(headingError * SERVO_P_GAIN,
                                             -(SERVO_NEUTRAL_US - SERVO_MIN_US),
                                             (SERVO_MAX_US - SERVO_NEUTRAL_US));
            servoPWM = SERVO_NEUTRAL_US + (int)servoDev;
        } else {
            float servoDev = constrainFloat(headingError * SERVO_P_GAIN,
                                             -(SERVO_NEUTRAL_US - SERVO_MIN_US),
                                             (SERVO_MAX_US - SERVO_NEUTRAL_US));
            servoPWM = SERVO_NEUTRAL_US + (int)servoDev;
        }

        Serial.printf("[PID] Dist: %5.1fm | Bearing: %6.1f | HeadErr: %6.1f\n",
                      distance, bearing, headingError);
        Serial.printf("[OUT] ESC: %4d us | Servo: %4d us | SpeedRatio: %.0f%%\n",
                      escPWM, servoPWM, yawErrorRatio * 100.0);
    }

    Serial.println("----------------------------------------\n");
}