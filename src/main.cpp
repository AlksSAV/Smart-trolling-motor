#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <TinyGPSPlus.h>
#include "ekf_heading.h"
#include "anchor_hold.h"      // <-- НОВОЕ
#include "esc_control.h"      // <-- НОВОЕ

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

#define RUN_STARTUP_TEST 0

// ============================================================
// Global Objects
// ============================================================
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
bool imu_ready = false;

const unsigned long IMU_TIMEOUT_MS = 1000;
unsigned long lastImuEventMs = 0;
bool imuHasData = false;

HardwareSerial gnssSerial(1);
TinyGPSPlus gps;
const unsigned long GNSS_BAUD = 38400;
uint32_t parsed_sentences = 0;

EkfHeading headingEKF;

float lastRawHeadingDeg = 0.0f;
bool  haveLastRawHeading = false;
unsigned long lastRawHeadingMs = 0;
float lastImuDeltaYawDeg = 0.0f;
float lastImuDtSec       = 0.0f;

const float GNSS_MIN_COG_MS    = 0.3f;
const float EKF_R_COG_DEG      = 10.0f;
const float EKF_INNOV_GATE_DEG = 60.0f;
uint32_t ekfRejectedCount = 0;
float lastEkfInnovDeg = 0.0f;

bool logTuningEnabled = false;

// ============================================================
// НОВЫЕ: Объекты управления
// ============================================================
AnchorHold    anchor;
ESCController esc_controller;

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
// Parameters
// ============================================================
const float LOITER_RADIUS_M    = 2.0;
const float MAX_DISTANCE_M     = 10.0;
const float SIM_DISTANCE_M     = 9.0;  // Simulated boat distance from target
const float THRUST_D_TAU_S     = 0.15; // Filtr time constant for the D term

// Control loop runs every 50 ms, but loop() jitters, so the real interval is
// measured. NOMINAL is the fallback for the very first call; MAX clamps an
// outlier step (blocked loop) so it cannot blow up the PID integral.
const float CONTROL_DT_NOMINAL_S = 0.05f;
const float CONTROL_DT_MAX_S     = 0.25f;

const int ESC_NEUTRAL_US       = 1500;

const int SERVO_NEUTRAL_US     = 1500;
const int SERVO_MIN_US         = 1000;
const int SERVO_MAX_US         = 2000;
const float SERVO_RANGE_DEG    = 90.0;
const float SERVO_DIR          = 1.0;

const float REVERSE_ENTER_DEG  = 100.0;
const float REVERSE_EXIT_DEG   = 80.0;

const float HEADING_DEADZONE_DEG     = 5.0;

const int LEDC_FREQ            = 50;
const int LEDC_RESOLUTION      = 14;

// ============================================================
// Math Helpers
// ============================================================
// Point destination from start given a bearing (deg) and distance (m).
void destinationPoint(double lat1, double lon1, float bearingDeg, float distM,
                      float &outLat, float &outLon) {
    const float R = 6371000.0f;
    float brng = bearingDeg * PI / 180.0f;
    float d = distM / R;
    double rLat = lat1 * PI / 180.0;
    double rLon = lon1 * PI / 180.0;
    double lat2 = asin(sin(rLat) * cos(d) + cos(rLat) * sin(d) * cos(brng));
    double lon2 = rLon + atan2(sin(brng) * sin(d) * cos(rLat),
                               cos(d) - sin(rLat) * sin(lat2));
    // lat2/lon2 are still radians - convert back, otherwise the caller's
    // degree-based distance/bearing math is off by 180/PI.
    outLat = (float)(lat2 * 180.0 / PI);
    outLon = (float)(lon2 * 180.0 / PI);
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

int angleToServoUs(float angleDeg) {
    angleDeg = constrainFloat(angleDeg, -SERVO_RANGE_DEG, SERVO_RANGE_DEG);
    return SERVO_NEUTRAL_US +
        (int)(SERVO_DIR * angleDeg * (SERVO_MAX_US - SERVO_NEUTRAL_US) / SERVO_RANGE_DEG);
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

float getCurrentHeading() {
    return fmod((360.0 - getCurrentYaw()), 360.0);
}

float getFusedHeading() {
    return headingEKF.heading();
}

void updateEkfFromImu(float rawHeadingDeg, unsigned long nowMs) {
    if (haveLastRawHeading) {
        float dt = (nowMs - lastRawHeadingMs) / 1000.0f;
        if (dt > 0.0f) {
            lastImuDeltaYawDeg = normalizeAngleDiff(rawHeadingDeg, lastRawHeadingDeg);
            lastImuDtSec = dt;
            headingEKF.predict(lastImuDeltaYawDeg, dt);
        }
    }
    lastRawHeadingDeg = rawHeadingDeg;
    lastRawHeadingMs  = nowMs;
    haveLastRawHeading = true;
}

void updateEkfFromGnss() {
    if (!imuHasData) return;
    if (!gps.course.isValid() || !gps.speed.isValid()) return;
    float speedMs = gps.speed.mps();
    if (speedMs < GNSS_MIN_COG_MS) return;

    if (currentMode == MODE_ANCHOR_HOLD && anchor.isThrustReversed()) return;

    float cog = gps.course.deg();
    float innov = normalizeAngleDiff(cog, headingEKF.heading());
    lastEkfInnovDeg = innov;

    if (!headingEKF.hasUpdate()) {
        headingEKF.seedHeading(cog);
    } else if (fabsf(innov) > EKF_INNOV_GATE_DEG) {
        ekfRejectedCount++;
    } else {
        headingEKF.correct(cog, EKF_R_COG_DEG);
    }
}

bool imuStale() {
    return !imu_ready || !imuHasData ||
           (millis() - lastImuEventMs > IMU_TIMEOUT_MS);
}

// ============================================================
// Control Output
// ============================================================
struct ControlState {
    float distance;
    float bearing;
    float headingError;
    float servoAngle;
    bool reversed;
    int escPWM;
    int servoPWM;
    bool gnssLost;
    bool imuLost;
};

void processSerialCommands();
ControlState computeControl(float currentYaw, float dt);
void calculateAndApplyControl(ControlState& ctrl, float dt);
void displaySystemStatus(const ControlState& ctrl);

// ============================================================
// Setup
// ============================================================
void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    Serial.println("\n========================================");
    Serial.println("  Smart Trolling Motor");
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
            lastImuEventMs = millis();
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

    // --- 3. Init LEDC ---
    Serial.println("\n[LEDC] Initializing PWM outputs...");
    ledcAttach(ESC_PIN, LEDC_FREQ, LEDC_RESOLUTION);
    ledcAttach(SERVO_PIN, LEDC_FREQ, LEDC_RESOLUTION);

    // --- 4. НОВОЕ: Инициализация контроллеров управления ---
    esc_controller.setRampRate(150.0f);
    esc_controller.setDeadzone(5.0f);
    esc_controller.setDirectPwm(ESC_NEUTRAL_US);

    anchor.setThrustTunings(20.0f, 0.1f, +10.0f); // D=+10 демпфирование (знак верен для PIDController)
    anchor.setThrustDerivativeTau(THRUST_D_TAU_S);
    anchor.setLoiterRadius(LOITER_RADIUS_M);
    anchor.setMaxDistance(MAX_DISTANCE_M);
    anchor.setReverseHysteresis(REVERSE_ENTER_DEG, REVERSE_EXIT_DEG);
    Serial.println("[CTRL] AnchorHold & ESCController initialized");

#if RUN_STARTUP_TEST
    Serial.println("\n========================================");
    Serial.println("  ESC TEST SEQUENCE");
    Serial.println("========================================");
    Serial.println("\n[ESC TEST] Step 1: Neutral (1500 us)");
    ledcWrite(ESC_PIN, usToDuty(ESC_NEUTRAL_US));
    delay(3000);
    Serial.println("\n[ESC TEST] Step 2: Small throttle (1600 us)");
    ledcWrite(ESC_PIN, usToDuty(1600));
    delay(2000);
    Serial.println("\n[ESC TEST] Step 3: Back to neutral (1500 us)");
    ledcWrite(ESC_PIN, usToDuty(ESC_NEUTRAL_US));
    delay(1000);
    Serial.println("\n[SERVO TEST] Moving servo through range...");
    ledcWrite(SERVO_PIN, usToDuty(SERVO_MIN_US));
    delay(1500);
    ledcWrite(SERVO_PIN, usToDuty(SERVO_NEUTRAL_US));
    delay(1500);
    ledcWrite(SERVO_PIN, usToDuty(SERVO_MAX_US));
    delay(1500);
    ledcWrite(SERVO_PIN, usToDuty(SERVO_NEUTRAL_US));
    delay(1000);
    Serial.println("\n========================================\n");
#else
    ledcWrite(ESC_PIN, usToDuty(ESC_NEUTRAL_US));
    ledcWrite(SERVO_PIN, usToDuty(SERVO_NEUTRAL_US));
    Serial.println("[SAFE START] ESC/Servo neutral (startup test disabled)");
#endif

    Serial.println(">>> Serial Commands:");
    Serial.println("    'M' = Manual");
    Serial.println("    'H' = Heading Hold");
    Serial.println("    'A' = Anchor Hold (NEW PID)");
    Serial.println("    'S' = Simulate Anchor");
    Serial.println("    'L' = Toggle CSV tuning log");
    Serial.println("========================================\n");
}

// ============================================================
// Main Loop
// ============================================================
void loop() {
    if (imu_ready) {
        if (bno08x.getSensorEvent(&sensorValue) &&
            sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
            lastImuEventMs = millis();
            imuHasData = true;

            static bool ekfInitialized = false;
            if (!ekfInitialized) {
                headingEKF.begin(getCurrentHeading());
                lastRawHeadingDeg = getCurrentHeading();
                lastRawHeadingMs = millis();
                haveLastRawHeading = true;
                ekfInitialized = true;
            }

            updateEkfFromImu(getCurrentHeading(), millis());
        }
    }

    while (gnssSerial.available() > 0) {
        if (gps.encode(gnssSerial.read())) {
            parsed_sentences++;
        }
    }

    if (gps.course.isUpdated()) {
        updateEkfFromGnss();
    }

    processSerialCommands();

    static unsigned long last_control = 0;
    static ControlState ctrlState;
    unsigned long now = millis();
    if (now - last_control > 50) {
        // dt is the real gap between control evaluations, not a nominal 50 ms:
        // Serial/I2C handling and FreeRTOS tasks make loop() jitter, and both
        // the thrust PID and ESCController integrate over this value.
        float dt = (last_control == 0) ? CONTROL_DT_NOMINAL_S
                                       : (now - last_control) / 1000.0f;
        // A blocked loop (GNSS dropout, long print) must not inflate the
        // integral or the derivative with an outlier step.
        if (dt <= 0.0f)   dt = CONTROL_DT_NOMINAL_S;
        if (dt > CONTROL_DT_MAX_S) dt = CONTROL_DT_MAX_S;

        calculateAndApplyControl(ctrlState, dt);
        last_control = now;
    }

    static unsigned long last_print = 0;
    if (millis() - last_print > 1000) {
        displaySystemStatus(ctrlState);
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
        anchor.deactivate();
        esc_controller.setDirectPwm(ESC_NEUTRAL_US);
        ledcWrite(ESC_PIN, usToDuty(ESC_NEUTRAL_US));
        ledcWrite(SERVO_PIN, usToDuty(SERVO_NEUTRAL_US));
        Serial.println(
"[MODE] MANUAL");
    }
    else if (cmd == 'H') {
        if (imuStale()) {
            Serial.println("\n[ERROR] IMU not ready or data stale!");
            return;
        }
        currentMode = MODE_HEADING_HOLD;
        isSimulatedTarget = false;
        anchor.deactivate();
        targetYaw = getFusedHeading();
        // Freeze the throttle the motor had when the mode was entered - from
        // now on HEADING_HOLD only steers, it never changes speed.
        anchor.setHeadingHoldBaseEsc(esc_controller.getCurrentPwm());
        Serial.printf("\n[MODE] HEADING HOLD. Target Yaw: %.1f deg | ESC frozen: %d us\n",
                      targetYaw, anchor.getHeadingHoldBaseEsc());
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
        anchor.setTarget((float)targetLat, (float)targetLon);
        anchor.activate();
        Serial.printf("\n[MODE] ANCHOR HOLD (NEW PID). Target: %.6f, %.6f\n", targetLat, targetLon);
    }
    else if (cmd == 'S') {
        currentMode = MODE_ANCHOR_HOLD;
        isSimulatedTarget = true;
        targetLat = 55.7558;
        targetLon = 37.6173;
        anchor.setTarget((float)targetLat, (float)targetLon);
        anchor.activate();
        Serial.println("\n[MODE] ANCHOR HOLD (SIMULATED, NEW PID). Target: 9m at 90 deg");
    }
    else if (cmd == 'L') {
        logTuningEnabled = !logTuningEnabled;
        Serial.printf("\n[LOG] Tuning CSV %s\n", logTuningEnabled ? "ON" : "OFF");
    }
}

// ============================================================
// Control Logic
// ============================================================
ControlState computeControl(float currentYaw, float dt) {
    ControlState out;
    out.distance = 0.0;
    out.bearing = 0.0;
    out.headingError = 0.0;
    out.servoAngle = 0.0;
    out.reversed = false;
    out.escPWM = ESC_NEUTRAL_US;
    out.servoPWM = SERVO_NEUTRAL_US;
    out.gnssLost = false;
    out.imuLost = false;

    if (currentMode != MODE_MANUAL && imuStale()) {
        out.imuLost = true;
        return out;
    }

    if (currentMode == MODE_MANUAL) {
        // ESC and servo stay at neutral
    }
    else if (currentMode == MODE_HEADING_HOLD) {
        float yawError = normalizeAngleDiff(targetYaw, currentYaw);
        out.headingError = yawError;
        // Steering only: throttle is frozen at the value captured on mode
        // entry, the servo rudders proportionally to the course error so it
        // tracks smoothly instead of snapping between the two end stops.
        if (fabsf(yawError) > HEADING_DEADZONE_DEG) {
            out.servoAngle = constrainFloat(yawError, -SERVO_RANGE_DEG, SERVO_RANGE_DEG);
        }
        out.escPWM = anchor.getHeadingHoldBaseEsc();
        out.servoPWM = angleToServoUs(out.servoAngle);
    }
    else if (currentMode == MODE_ANCHOR_HOLD) {
        if (!isSimulatedTarget && !gps.location.isValid()) {
            out.gnssLost = true;
            anchor.deactivate();
            return out;
        }

        // In simulation the boat is pinned 9 m due west of the target, so the
        // target always lies 9 m ahead at bearing 90 deg. Feeding the target
        // itself as the current position (previous behaviour) made the distance
        // zero and the PID never produced thrust.
        float lat, lon;
        if (isSimulatedTarget) {
            destinationPoint(targetLat, targetLon, 270.0f, SIM_DISTANCE_M, lat, lon);
        } else {
            lat = (float)gps.location.lat();
            lon = (float)gps.location.lng();
        }
        float speed_ms = (gps.speed.isValid()) ? gps.speed.mps() : 0.0f;

        ControlOutput cmd = anchor.update(lat, lon, currentYaw, speed_ms, dt);

        out.servoAngle = cmd.servo_angle;
        out.reversed   = anchor.isThrustReversed();

        out.escPWM = esc_controller.getSmoothedPwm(cmd.thrust);
        out.servoPWM = angleToServoUs(out.servoAngle);

        out.distance = gps.distanceBetween(lat, lon, targetLat, targetLon);
        out.bearing = anchor.getBearing(lat, lon, targetLat, targetLon);
        out.headingError = normalizeAngleDiff(out.bearing, currentYaw);
    }

    return out;
}

void calculateAndApplyControl(ControlState& ctrl, float dt) {
    ctrl = computeControl(imuStale() ? 0.0f : getFusedHeading(), dt);

    ledcWrite(ESC_PIN, usToDuty(ctrl.escPWM));
    // Outside the smoothed path the ramp state must follow the written PWM,
    // otherwise the next ANCHOR_HOLD entry ramps from a stale current_pwm.
    if (!(currentMode == MODE_ANCHOR_HOLD && !ctrl.gnssLost && !ctrl.imuLost)) {
        esc_controller.setDirectPwm(ctrl.escPWM);
    }
    ledcWrite(SERVO_PIN, usToDuty(ctrl.servoPWM));

    if (logTuningEnabled && imuHasData && haveLastRawHeading) {
        Serial.printf("LOG,%lu,%d,%6.1f,%6.1f,%5.2f,%lu,%d,%d,%.1f,%.2f,%.6f,%.6f,%.1f,%.1f,%6.1f,%6.3f,%6.1f,%u,%.1f,%.1f\n",
            millis(), currentMode,
            getCurrentHeading(), lastImuDeltaYawDeg, lastImuDtSec, millis() - lastImuEventMs,
            gps.location.isValid() ? 1 : 0, gps.satellites.value(),
            (gps.course.isValid() && gps.speed.isValid()) ? gps.course.deg() : 0.0f,
            (gps.speed.isValid()) ? gps.speed.mps() : 0.0f,
            gps.location.lat(), gps.location.lng(),
            (currentMode == MODE_ANCHOR_HOLD && !isSimulatedTarget && gps.location.isValid())
                ? gps.distanceBetween(gps.location.lat(), gps.location.lng(), targetLat, targetLon) : 0.0f,
            (currentMode == MODE_ANCHOR_HOLD && !isSimulatedTarget && gps.location.isValid())
                ? anchor.getBearing(gps.location.lat(), gps.location.lng(), targetLat, targetLon) : 0.0f,
            headingEKF.heading(),
            headingEKF.biasDegPerSec(),
            lastEkfInnovDeg,
            ekfRejectedCount,
            ctrl.servoAngle,
            ctrl.reversed ? -1.0f : 1.0f);
    }
}

// ============================================================
// Status Display
// ============================================================
void displaySystemStatus(const ControlState& ctrl) {
    Serial.println("----------------------------------------");
    String modeStr;
    switch (currentMode) {
        case MODE_MANUAL:       modeStr = "MANUAL";       break;
        case MODE_HEADING_HOLD: modeStr = "HEADING HOLD"; break;
        case MODE_ANCHOR_HOLD:  modeStr = "ANCHOR HOLD";  break;
    }
    if (isSimulatedTarget) modeStr += " (SIM)";
    Serial.printf("[SYS] Mode: %-16s\n", modeStr.c_str());

    float currentHeading = 0.0;
    if (!imu_ready) {
        Serial.println("[IMU] NOT READY");
    } else if (imuStale()) {
        Serial.println("[IMU] STALE (no data > 1000 ms)");
    } else {
        currentHeading = getCurrentHeading();
        Serial.printf("[IMU] Yaw: %6.1f | Heading: %6.1f deg\n",
                      getCurrentYaw(), currentHeading);
    }

    uint32_t sentencesPerSec = parsed_sentences;
    parsed_sentences = 0;
    Serial.printf("[GNSS] %lu msg/s | Sats: %2d | Fix: %s\n",
                  (unsigned long)sentencesPerSec, gps.satellites.value(),
                  gps.location.isValid() ? "YES" : "NO");

    if (currentMode == MODE_HEADING_HOLD || currentMode == MODE_ANCHOR_HOLD) {
        if (ctrl.imuLost) {
            Serial.println("[WARN] IMU data stale - ESC/Servo neutral");
        } else if (currentMode == MODE_HEADING_HOLD) {
            Serial.printf("[OUT] Target: %6.1f deg | Err: %6.1f deg | ESC: %4d us | Servo: %4d us (%5.1f deg)\n",
                          targetYaw, ctrl.headingError, ctrl.escPWM,
                          ctrl.servoPWM, ctrl.servoAngle);
        } else if (ctrl.gnssLost) {
            Serial.println("[WARN] ANCHOR HOLD: no GNSS fix - ESC/Servo neutral");
        } else {
            Serial.printf("[PID] Dist: %5.1fm | Bearing: %6.1f | OffBow: %6.1f\n",
                          ctrl.distance, ctrl.bearing, ctrl.headingError);
            Serial.printf("[OUT] ESC: %4d us (%s) | Servo: %4d us (%5.1f deg)\n",
                          ctrl.escPWM, ctrl.reversed ? "REV" : "FWD",
                          ctrl.servoPWM, ctrl.servoAngle);
        }
    }
    Serial.println("----------------------------------------\n");
}