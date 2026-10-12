#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <TinyGPSPlus.h>
#include "ekf_heading.h"

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
// IMU reports at 20 Hz (50 ms). No new rotation vector for this long = stale.
const unsigned long IMU_TIMEOUT_MS = 1000;
unsigned long lastImuEventMs = 0;
// False until the first rotation vector arrives: sensorValue is zeroed before
// that, and getCurrentYaw() would return a bogus 0 deg.
bool imuHasData = false;

HardwareSerial gnssSerial(1);
TinyGPSPlus gps;
const unsigned long GNSS_BAUD = 38400;
uint32_t parsed_sentences = 0;

// Heading EKF — fused heading output.
EkfHeading headingEKF;
// Last IMU increment (for the logger).
float lastRawHeadingDeg = 0.0f;
bool  haveLastRawHeading = false;
unsigned long lastRawHeadingMs = 0;
float lastImuDeltaYawDeg = 0.0f;
float lastImuDtSec       = 0.0f;
// EKF tuning / policy constants.
// COG speed threshold below which COG is ignored (m/s).
const float GNSS_MIN_COG_MS  = 0.3f;
// 1-sigma of COG as a heading measurement (placeholder, tune from log).
const float EKF_R_COG_DEG    = 10.0f;
// Innovation gate: reject measurements further than this from the estimate.
const float EKF_INNOV_GATE_DEG = 60.0f;
// Counter for rejected measurements (diagnostic).
uint32_t ekfRejectedCount = 0;
// Last innovation for logging (both accepted and rejected).
float lastEkfInnovDeg = 0.0f;
// CSV tuning log: enabled by 'L' command.
bool logTuningEnabled = false;

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
const float LOITER_SPEED_GAIN  = 0.15;

// ESC is bidirectional: 1500 neutral, >1500 forward, <1500 reverse.
// It ignores 1500 +/- ESC_DEADBAND_US, so speed 0..1 is mapped outside the band.
const int ESC_NEUTRAL_US       = 1500;
const int ESC_DEADBAND_US      = 50;
const int ESC_MAX_FORWARD_US   = 1800;
const int ESC_MAX_REVERSE_US   = 1250;

// Servo: 1000..2000 us = -90..+90 deg from the boat centerline (linear).
const int SERVO_NEUTRAL_US     = 1500;
const int SERVO_MIN_US         = 1000;
const int SERVO_MAX_US         = 2000;
const float SERVO_RANGE_DEG    = 90.0;
// +1: positive angle -> longer pulse. Set to -1 if the servo turns the wrong
// way on the bench. Also depends on the yaw sign convention (not verified).
const float SERVO_DIR          = 1.0;

// ANCHOR_HOLD: thrust switches to reverse when the target is more than
// REVERSE_ENTER_DEG off the bow, back to forward below REVERSE_EXIT_DEG.
const float REVERSE_ENTER_DEG  = 100.0;
const float REVERSE_EXIT_DEG   = 80.0;

// HEADING_HOLD: the servo is turned fully to the side the heading must change
// toward; thrust grows with the error. Starting values, not tuned.
const float HEADING_DEADZONE_DEG     = 5.0;
const float HEADING_FULL_THRUST_DEG  = 45.0;
const float HEADING_MAX_SPEED        = 0.5;

const int LEDC_FREQ            = 50;
const int LEDC_RESOLUTION      = 14;

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

// speed: -1.0 (full reverse) .. +1.0 (full forward), 0 = neutral.
int speedToEscUs(float speed) {
    speed = constrainFloat(speed, -1.0, 1.0);
    if (speed > 0.0) {
        int start = ESC_NEUTRAL_US + ESC_DEADBAND_US;
        return start + (int)(speed * (ESC_MAX_FORWARD_US - start));
    }
    if (speed < 0.0) {
        int start = ESC_NEUTRAL_US - ESC_DEADBAND_US;
        return start - (int)(-speed * (start - ESC_MAX_REVERSE_US));
    }
    return ESC_NEUTRAL_US;
}

// angleDeg: -90..+90 relative to the boat centerline.
int angleToServoUs(float angleDeg) {
    angleDeg = constrainFloat(angleDeg, -SERVO_RANGE_DEG, SERVO_RANGE_DEG);
    return SERVO_NEUTRAL_US +
           (int)(SERVO_DIR * angleDeg * (SERVO_MAX_US - SERVO_NEUTRAL_US) / SERVO_RANGE_DEG);
}

// Hysteresis latch for ANCHOR_HOLD thrust direction (reset on mode change).
bool thrustReversed = false;

void updateThrustDirection(float targetOffBowDeg) {
    float a = fabsf(targetOffBowDeg);
    if (!thrustReversed && a > REVERSE_ENTER_DEG) {
        thrustReversed = true;
    } else if (thrustReversed && a < REVERSE_EXIT_DEG) {
        thrustReversed = false;
    }
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

// BNO085 yaw grows counter-clockwise, GNSS bearing and the servo angle grow
// clockwise. Verified on hardware 2026-10-12: one full clockwise turn gave
// -360 deg of yaw (N 117.8, E 36.8, S 293.1, W 198.7, back to N). Yaw is
// mirrored here so both are compared in the same direction.
// A constant offset (mounting + magnetic declination) remains and is NOT
// applied yet: it cannot be separated from magnetic distortion indoors.
float getCurrentHeading() {
    return fmod((360.0 - getCurrentYaw()), 360.0);
}

// Fused heading from the EKF. Before the first GNSS correction the filter
// output equals the raw IMU heading, so this is safe to use everywhere.
float getFusedHeading() {
    return headingEKF.heading();
}

// Called on every new rotation vector: integrate the raw heading increment.
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

// Called once per GNSS solution (5 Hz). Feeds the EKF a heading measurement.
// The ONLY valid GNSS heading measurement is COG (Course Over Ground), and only
// when the boat is actually moving. Bearing to target is NOT a heading measurement:
// it is the direction from the boat to the anchor, which tells nothing about the
// boat's heading. At standstill there is no valid GNSS heading measurement.
// When thrust is reversed (ANCHOR_HOLD), COG points 180 deg from the bow: skip.
void updateEkfFromGnss() {
    if (!imuHasData) return;
    if (!gps.course.isValid() || !gps.speed.isValid()) return;

    float speedMs = gps.speed.mps();
    if (speedMs < GNSS_MIN_COG_MS) return;

    // Reverse thrust: COG points where the boat moves, not where the bow points.
    if (currentMode == MODE_ANCHOR_HOLD && thrustReversed) return;

    float cog = gps.course.deg();
    float innov = normalizeAngleDiff(cog, headingEKF.heading());
    lastEkfInnovDeg = innov;

    if (!headingEKF.hasUpdate()) {
        // Not yet aligned. Seed the estimate to the measurement and try again
        // next cycle. After two or three cycles the innovation falls within the
        // gate and correct() marks the filter converged.
        headingEKF.seedHeading(cog);
    } else if (fabsf(innov) > EKF_INNOV_GATE_DEG) {
        // Outlier: bad fix or aggressive maneuver. Reject.
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
// Control Output (shared by control loop and status display)
// ============================================================
struct ControlState {
    float distance;      // m (ANCHOR_HOLD only)
    float bearing;       // deg (ANCHOR_HOLD only)
    float headingError;  // deg (HEADING_HOLD: yaw error, ANCHOR_HOLD: target off the bow)
    float servoAngle;    // deg, -90..+90 from the centerline
    bool reversed;       // ANCHOR_HOLD: thrust is backward
    int escPWM;          // us
    int servoPWM;        // us
    bool gnssLost;       // ANCHOR_HOLD: no valid GNSS fix, outputs forced neutral
    bool imuLost;        // HEADING/ANCHOR_HOLD: IMU data stale, outputs forced neutral
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

    // GNSS module is configured once via tools/gnss_config.py (stored in
    // module Flash); firmware does not write to it.

    // --- 3. Init LEDC for ESC and Servo ---
    Serial.println("\n[LEDC] Initializing PWM outputs...");

    ledcAttach(ESC_PIN, LEDC_FREQ, LEDC_RESOLUTION);
    ledcAttach(SERVO_PIN, LEDC_FREQ, LEDC_RESOLUTION);

#if RUN_STARTUP_TEST
    // --- 4. ESC TEST SEQUENCE (bench only) ---
    Serial.println("\n========================================");
    Serial.println("  ESC TEST SEQUENCE");
    Serial.println("  Listen for ESC beeps!");
    Serial.println("========================================");

    // Step 1: Neutral (ESC should beep once to acknowledge)
    Serial.println("\n[ESC TEST] Step 1: Neutral (1500 us)");
    Serial.println(">>> ESC should beep ONCE <<<");
    ledcWrite(ESC_PIN, usToDuty(ESC_NEUTRAL_US));
    delay(3000);

    // Step 2: Small throttle forward (motor should spin slowly if connected).
    // 1000 us is NOT used here: on a bidirectional ESC it is full reverse.
    Serial.println("\n[ESC TEST] Step 2: Small throttle (1600 us)");
    Serial.println(">>> If motor connected, it should spin slowly <<<");
    ledcWrite(ESC_PIN, usToDuty(1600));
    delay(2000);

    // Step 3: Back to neutral
    Serial.println("\n[ESC TEST] Step 3: Back to neutral (1500 us)");
    ledcWrite(ESC_PIN, usToDuty(ESC_NEUTRAL_US));
    delay(1000);

    // Step 4: Servo test
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

    // --- 5. Print help ---
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
        if (bno08x.getSensorEvent(&sensorValue) &&
            sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
            lastImuEventMs = millis();
            imuHasData = true;

            // Lazy init: begin EKF on first valid IMU data.
            static bool ekfInitialized = false;
            if (!ekfInitialized) {
                headingEKF.begin(getCurrentHeading());
                lastRawHeadingDeg = getCurrentHeading();
                lastRawHeadingMs = millis();
                haveLastRawHeading = true;
                ekfInitialized = true;
            }

            // EKF predict step.
            updateEkfFromImu(getCurrentHeading(), millis());
        }
    }

    while (gnssSerial.available() > 0) {
        if (gps.encode(gnssSerial.read())) {
            parsed_sentences++;
        }
    }

    // EKF correct step on every new GNSS solution.
    if (gps.course.isUpdated()) {
        updateEkfFromGnss();
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
        if (imuStale()) {
            Serial.println("\n[ERROR] IMU not ready or data stale!");
            return;
        }
        currentMode = MODE_HEADING_HOLD;
        isSimulatedTarget = false;
        targetYaw = getFusedHeading();
        Serial.printf("\n[MODE] HEADING HOLD. Target Yaw: %.1f deg\n", targetYaw);
    }
    else if (cmd == 'A') {
        if (!gps.location.isValid()) {
            Serial.println("\n[ERROR] No GNSS fix. Use 'S' to simulate indoors.");
            return;
        }
        currentMode = MODE_ANCHOR_HOLD;
        isSimulatedTarget = false;
        thrustReversed = false;
        targetLat = gps.location.lat();
        targetLon = gps.location.lng();
        Serial.printf("\n[MODE] ANCHOR HOLD. Target: %.6f, %.6f\n", targetLat, targetLon);
    }
    else if (cmd == 'S') {
        currentMode = MODE_ANCHOR_HOLD;
        isSimulatedTarget = true;
        thrustReversed = false;
        targetLat = 55.7558;
        targetLon = 37.6173;
        Serial.println("\n[MODE] ANCHOR HOLD (SIMULATED). Target: 9m at 90 deg");
    }
    else if (cmd == 'L') {
        logTuningEnabled = !logTuningEnabled;
        Serial.printf("\n[LOG] Tuning CSV %s\n", logTuningEnabled ? "ON" : "OFF");
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
    out.servoAngle = 0.0;
    out.reversed = false;
    out.escPWM = ESC_NEUTRAL_US;
    out.servoPWM = SERVO_NEUTRAL_US;
    out.gnssLost = false;
    out.imuLost = false;

    // Stale IMU yaw would steer on old heading: hold neutral instead.
    if (currentMode != MODE_MANUAL && imuStale()) {
        out.imuLost = true;
        return out;
    }

    if (currentMode == MODE_MANUAL) {
        // ESC and servo stay at neutral
    }
    else if (currentMode == MODE_HEADING_HOLD) {
        // Bow motor turns the boat by pushing the bow sideways: servo fully to
        // the side of the error, thrust grows with the error size.
        float yawError = normalizeAngleDiff(targetYaw, currentYaw);
        out.headingError = yawError;
        float absErr = fabsf(yawError);
        if (absErr > HEADING_DEADZONE_DEG) {
            float speed = constrainFloat(absErr / HEADING_FULL_THRUST_DEG, 0.0, 1.0)
                          * HEADING_MAX_SPEED;
            out.servoAngle = (yawError > 0.0) ? SERVO_RANGE_DEG : -SERVO_RANGE_DEG;
            out.escPWM = speedToEscUs(speed);
        }
        out.servoPWM = angleToServoUs(out.servoAngle);
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

        // Direction to the target relative to the bow, -180..+180.
        float d = normalizeAngleDiff(out.bearing, currentYaw);
        out.headingError = d;

        // Thrust forward with the servo at d, or backward with the servo
        // turned 180 deg away so that the reverse thrust still points at d.
        updateThrustDirection(d);
        out.reversed = thrustReversed;
        if (thrustReversed) {
            out.servoAngle = (d > 0.0) ? d - 180.0 : d + 180.0;
        } else {
            out.servoAngle = d;
        }

        if (out.distance > LOITER_RADIUS_M) {
            float distanceError = out.distance - LOITER_RADIUS_M;
            float speed = constrainFloat(
                distanceError / (MAX_DISTANCE_M - LOITER_RADIUS_M), 0.0, 1.0);
            out.escPWM = speedToEscUs(thrustReversed ? -speed : speed);
        }
        out.servoPWM = angleToServoUs(out.servoAngle);
    }

    return out;
}

void calculateAndApplyControl() {
    // computeControl() returns neutral when the IMU is stale or missing,
    // so outputs are always written (never left at the last value).
    ControlState ctrl = computeControl(imuStale() ? 0.0 : getFusedHeading());
    ledcWrite(ESC_PIN, usToDuty(ctrl.escPWM));
    ledcWrite(SERVO_PIN, usToDuty(ctrl.servoPWM));

    // CSV tuning log at 20 Hz (only when enabled).
    if (logTuningEnabled && imuHasData && haveLastRawHeading) {
        Serial.printf("LOG,%lu,%d,%6.1f,%6.1f,%5.2f,%lu,%d,%d,%.1f,%.2f,%.6f,%.6f,%.1f,%.1f,%6.1f,%6.3f,%6.1f,%u\n",
            millis(), currentMode,
            getCurrentHeading(), lastImuDeltaYawDeg, lastImuDtSec, millis() - lastImuEventMs,
            gps.location.isValid() ? 1 : 0, gps.satellites.value(),
            (gps.course.isValid() && gps.speed.isValid()) ? gps.course.deg() : 0.0f,
            (gps.speed.isValid()) ? gps.speed.mps() : 0.0f,
            gps.location.lat(), gps.location.lng(),
            (currentMode == MODE_ANCHOR_HOLD && !isSimulatedTarget && gps.location.isValid())
                ? gps.distanceBetween(gps.location.lat(), gps.location.lng(), targetLat, targetLon) : 0.0f,
            (currentMode == MODE_ANCHOR_HOLD && !isSimulatedTarget && gps.location.isValid())
                ? calculateBearing(gps.location.lat(), gps.location.lng(), targetLat, targetLon) : 0.0f,
            headingEKF.heading(),
            headingEKF.biasDegPerSec(),
            lastEkfInnovDeg,
            ekfRejectedCount);
    }
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

    // Status is printed once per second, so the counter equals sentences/s.
    uint32_t sentencesPerSec = parsed_sentences;
    parsed_sentences = 0;
    Serial.printf("[GNSS] %lu msg/s | Sats: %2d | Fix: %s\n",
                  (unsigned long)sentencesPerSec, gps.satellites.value(),
                  gps.location.isValid() ? "YES" : "NO");

    if (currentMode == MODE_HEADING_HOLD || currentMode == MODE_ANCHOR_HOLD) {
        ControlState ctrl = computeControl(getFusedHeading());

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