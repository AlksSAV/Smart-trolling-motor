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

#define GNSS_RX         18
#define GNSS_TX         17

#define ESC_PIN         15
#define SERVO_PIN       16

// Startup ESC/servo test sequence spins the motor. Keep 0 on the water:
// set to 1 only for bench testing (causes motor jerk on every reset).
#define RUN_STARTUP_TEST 0

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
const float LOITER_RADIUS_M    = 2.0;
const float MAX_DISTANCE_M     = 10.0;
const float SERVO_P_GAIN       = 8.0;
const float LOITER_SPEED_GAIN  = 0.15;

const int ESC_NEUTRAL_US       = 1500;
const int ESC_MAX_FORWARD_US   = 1800;
const int SERVO_NEUTRAL_US     = 1500;
const int SERVO_MIN_US         = 1000;
const int SERVO_MAX_US         = 2000;

const int LEDC_FREQ            = 50;
const int LEDC_RESOLUTION      = 14;

// ============================================================
// UBX Helper
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

uint32_t usToDuty(uint32_t us) {
    return (us * 16384) / 20000;
}

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
// Control Output (shared by control loop and status display)
// ============================================================
struct ControlState {
    float distance;      // m (ANCHOR_HOLD only)
    float bearing;       // deg (ANCHOR_HOLD only)
    float headingError;  // deg (HEADING_HOLD: yaw error)
    float yawErrorRatio; // 1.0 .. 0.5 (ANCHOR_HOLD only)
    int escPWM;          // us
    int servoPWM;        // us
    bool gnssLost;       // ANCHOR_HOLD: no valid GNSS fix, outputs forced neutral
};

void processSerialCommands();
ControlState computeControl(float currentYaw);
void calculateAndApplyControl();
void displaySystemStatus();

// ============================================================
// Setup
// ============================================================
void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }

    Serial.println("\n========================================");
    Serial.println("  Smart Trolling Motor v0.3.1");
    Serial.println("  ESC Test Sequence Added");
    Serial.println("========================================");
    Serial.println("Waiting 2 seconds for modules to power up...");
    delay(2000);

    // --- 1. Init IMU ---
    Serial.println("\n[IMU] Initializing BNO085...");
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);

    int imu_attempts = 0;
    while (!imu_ready && imu_attempts < 3) {
        imu_attempts++;
        Serial.printf("[IMU] Attempt %d/3...\n", imu_attempts);
        if (bno08x.begin_I2C(BNO08X_ADDRESS)) {
            bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 50000);
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

    // --- 2. Init GNSS ---
    Serial.println("\n[GNSS] Initializing UART1...");
    gnssSerial.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX, GNSS_TX);
    Serial.printf("[GNSS] OK: Listening at %lu baud.\n", GNSS_BAUD);

    // --- 3. Send UBX Config ---
    if (imu_ready) {
        Serial.println("[GNSS] Sending UBX config: 5Hz + Sea mode...");
        delay(500);

        uint8_t ratePayload[] = {0xC8, 0x00, 0x01, 0x00, 0x00, 0x00};
        sendUBX(0x06, 0x08, ratePayload, sizeof(ratePayload));

        uint8_t nav5Payload[36] = {0};
        nav5Payload[0] = 0x01;
        nav5Payload[1] = 0x00;
        nav5Payload[2] = 0x06;
        sendUBX(0x06, 0x24, nav5Payload, sizeof(nav5Payload));

        ubx_configured = true;
        Serial.println("[GNSS] OK: Configuration applied.");
    }

    // --- 4. Init LEDC for ESC and Servo ---
    Serial.println("\n[LEDC] Initializing PWM outputs...");

    ledcAttach(ESC_PIN, LEDC_FREQ, LEDC_RESOLUTION);
    ledcAttach(SERVO_PIN, LEDC_FREQ, LEDC_RESOLUTION);

#if RUN_STARTUP_TEST
    // --- 5. ESC TEST SEQUENCE (bench only) ---
    Serial.println("\n========================================");
    Serial.println("  ESC TEST SEQUENCE");
    Serial.println("  Listen for ESC beeps!");
    Serial.println("========================================");

    // Step 1: Neutral (ESC should beep once to acknowledge)
    Serial.println("\n[ESC TEST] Step 1: Neutral (1500 us)");
    Serial.println(">>> ESC should beep ONCE <<<");
    ledcWrite(ESC_PIN, usToDuty(ESC_NEUTRAL_US));
    delay(3000);

    // Step 2: Arm (some ESCs need this)
    Serial.println("\n[ESC TEST] Step 2: Minimum throttle (1000 us) for 2 sec");
    Serial.println(">>> ESC should beep to confirm arming <<<");
    ledcWrite(ESC_PIN, usToDuty(1000));
    delay(2000);

    // Step 3: Back to neutral
    Serial.println("\n[ESC TEST] Step 3: Back to neutral (1500 us)");
    ledcWrite(ESC_PIN, usToDuty(ESC_NEUTRAL_US));
    delay(1000);

    // Step 4: Small throttle increase (motor should spin slowly if connected)
    Serial.println("\n[ESC TEST] Step 4: Small throttle (1550 us)");
    Serial.println(">>> If motor connected, it should spin slowly <<<");
    ledcWrite(ESC_PIN, usToDuty(1550));
    delay(2000);

    // Step 5: Back to neutral
    Serial.println("\n[ESC TEST] Step 5: Back to neutral (1500 us)");
    ledcWrite(ESC_PIN, usToDuty(ESC_NEUTRAL_US));
    delay(1000);

    // Step 6: Servo test
    Serial.println("\n[SERVO TEST] Moving servo through range...");
    ledcWrite(SERVO_PIN, usToDuty(SERVO_MIN_US));
    Serial.println("[SERVO] Position: 1000 us (MIN)");
    delay(1500);

    ledcWrite(SERVO_PIN, usToDuty(SERVO_NEUTRAL_US));
    Serial.println("[SERVO] Position: 1500 us (NEUTRAL)");
    delay(1500);

    ledcWrite(SERVO_PIN, usToDuty(SERVO_MAX_US));
    Serial.println("[SERVO] Position: 2000 us (MAX)");
    delay(1500);

    ledcWrite(SERVO_PIN, usToDuty(SERVO_NEUTRAL_US));
    Serial.println("[SERVO] Back to neutral");
    delay(1000);

    Serial.println("\n========================================");
    Serial.println("  TEST COMPLETE");
    Serial.println("  Starting normal operation...");
    Serial.println("========================================\n");
#else
    // Safe start: hold both outputs at neutral, no motor spin.
    ledcWrite(ESC_PIN, usToDuty(ESC_NEUTRAL_US));
    ledcWrite(SERVO_PIN, usToDuty(SERVO_NEUTRAL_US));
    Serial.println("[SAFE START] ESC/Servo neutral (startup test disabled)");
#endif

    // --- 6. Print help ---
    Serial.println(">>> Serial Commands:");
    Serial.println("    'M' = Manual");
    Serial.println("    'H' = Heading Hold");
    Serial.println("    'A' = Anchor Hold");
    Serial.println("    'S' = Simulate Anchor");
    Serial.println("========================================\n");
}

// ============================================================
// Main Loop
// ============================================================
void loop() {
    if (imu_ready) {
        bno08x.getSensorEvent(&sensorValue);
    }

    while (gnssSerial.available() > 0) {
        if (gps.encode(gnssSerial.read())) {
            parsed_sentences++;
        }
    }

    processSerialCommands();

    static unsigned long last_control = 0;
    if (millis() - last_control > 50) {
        calculateAndApplyControl();
        last_control = millis();
    }

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
        targetLat = 55.7558;
        targetLon = 37.6173;
        Serial.println("\n[MODE] ANCHOR HOLD (SIMULATED). Target: 9m at 90 deg");
    }
}

// ============================================================
// ArduPilot-inspired Control Logic
// ============================================================
ControlState computeControl(float currentYaw) {
    ControlState out;
    out.distance = 0.0;
    out.bearing = 0.0;
    out.headingError = 0.0;
    out.yawErrorRatio = 1.0;
    out.escPWM = ESC_NEUTRAL_US;
    out.servoPWM = SERVO_NEUTRAL_US;
    out.gnssLost = false;

    if (currentMode == MODE_MANUAL) {
        // ESC and servo stay at neutral
    }
    else if (currentMode == MODE_HEADING_HOLD) {
        float yawError = normalizeAngleDiff(targetYaw, currentYaw);
        out.headingError = yawError;
        float servoDev = constrainFloat(yawError * SERVO_P_GAIN,
                                         -(SERVO_NEUTRAL_US - SERVO_MIN_US),
                                         (SERVO_MAX_US - SERVO_NEUTRAL_US));
        out.servoPWM = SERVO_NEUTRAL_US + (int)servoDev;
    }
    else if (currentMode == MODE_ANCHOR_HOLD) {
        // No valid fix (and not simulating): hold safe neutral instead of
        // steering on garbage bearing. ESC/servo already neutral.
        if (!isSimulatedTarget && !gps.location.isValid()) {
            out.gnssLost = true;
            return out;
        }

        if (isSimulatedTarget) {
            out.distance = 9.0;
            out.bearing = 90.0;
        } else if (gps.location.isValid()) {
            out.distance = gps.distanceBetween(
                gps.location.lat(), gps.location.lng(),
                targetLat, targetLon);
            out.bearing = calculateBearing(
                gps.location.lat(), gps.location.lng(),
                targetLat, targetLon);
        }

        out.headingError = normalizeAngleDiff(out.bearing, currentYaw);
        out.yawErrorRatio = 1.0 - (constrainFloat(fabsf(out.headingError), 0.0, 90.0) / 90.0) * 0.5;

        if (out.distance > LOITER_RADIUS_M) {
            float distanceError = out.distance - LOITER_RADIUS_M;
            float speedFactor = constrainFloat(
                distanceError / (MAX_DISTANCE_M - LOITER_RADIUS_M), 0.0, 1.0);
            float effectiveSpeed = speedFactor * out.yawErrorRatio;
            out.escPWM = ESC_NEUTRAL_US + (int)(effectiveSpeed * (ESC_MAX_FORWARD_US - ESC_NEUTRAL_US));
        }

        float servoDev = constrainFloat(out.headingError * SERVO_P_GAIN,
                                         -(SERVO_NEUTRAL_US - SERVO_MIN_US),
                                         (SERVO_MAX_US - SERVO_NEUTRAL_US));
        out.servoPWM = SERVO_NEUTRAL_US + (int)servoDev;
    }

    return out;
}

void calculateAndApplyControl() {
    if (!imu_ready) return;

    ControlState ctrl = computeControl(getCurrentYaw());
    ledcWrite(ESC_PIN, usToDuty(ctrl.escPWM));
    ledcWrite(SERVO_PIN, usToDuty(ctrl.servoPWM));
}

// ============================================================
// Status Display
// ============================================================
void displaySystemStatus() {
    Serial.println("----------------------------------------");

    String modeStr;
    switch (currentMode) {
        case MODE_MANUAL:       modeStr = "MANUAL";       break;
        case MODE_HEADING_HOLD: modeStr = "HEADING HOLD"; break;
        case MODE_ANCHOR_HOLD:  modeStr = "ANCHOR HOLD";  break;
    }
    if (isSimulatedTarget) modeStr += " (SIM)";
    Serial.printf("[SYS] Mode: %-16s\n", modeStr.c_str());

    float currentYaw = 0.0;
    if (imu_ready) {
        currentYaw = getCurrentYaw();
        Serial.printf("[IMU] Yaw: %6.1f deg\n", currentYaw);
    } else {
        Serial.println("[IMU] NOT READY");
    }

    float hz = (parsed_sentences * 1000.0) / 1000.0;
    parsed_sentences = 0;
    Serial.printf("[GNSS] ~%.0f Hz | Sats: %2d | Fix: %s\n",
                  hz, gps.satellites.value(),
                  gps.location.isValid() ? "YES" : "NO");

    if (imu_ready && (currentMode == MODE_HEADING_HOLD || currentMode == MODE_ANCHOR_HOLD)) {
        ControlState ctrl = computeControl(currentYaw);

        if (currentMode == MODE_HEADING_HOLD) {
            Serial.printf("[OUT] Target: %6.1f deg | Err: %6.1f deg | Servo: %4d us\n",
                          targetYaw, ctrl.headingError, ctrl.servoPWM);
        } else if (ctrl.gnssLost) {
            Serial.println("[WARN] ANCHOR HOLD: no GNSS fix - ESC/Servo neutral");
        } else {
            Serial.printf("[PID] Dist: %5.1fm | Bearing: %6.1f | HeadErr: %6.1f\n",
                          ctrl.distance, ctrl.bearing, ctrl.headingError);
            Serial.printf("[OUT] ESC: %4d us | Servo: %4d us | SpeedRatio: %.0f%%\n",
                          ctrl.escPWM, ctrl.servoPWM, ctrl.yawErrorRatio * 100.0);
        }
    }

    Serial.println("----------------------------------------\n");
}