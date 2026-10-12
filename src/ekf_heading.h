#ifndef EKF_HEADING_H
#define EKF_HEADING_H

#include <Arduino.h>

// Heading EKF: state = [yaw, gyro_bias].
//
// yaw   - fused heading, degrees, 0..360, grows clockwise (same convention as
//         GNSS bearing and the servo angle)
// bias  - slow drift of the BNO085 heading, degrees per second
//
// predict() runs on every IMU rotation vector (20 Hz) and integrates the raw
// yaw increment with the drift estimate subtracted.
// correct() runs when a usable GNSS heading measurement arrives (5 Hz) and
// pulls yaw and bias toward it.
//
// Q and R are placeholders until measured from a log (tools/). Values here
// only keep the filter numerically sane.

class EkfHeading {
public:
    EkfHeading();

    // Raw yaw at startup seeds the state; everything else starts untrained.
    void begin(float initialYawDeg);

    // IMU step. dYawDeg is the raw heading increment since the previous call,
    // normalized to -180..180. dtSec is the elapsed time in seconds.
    void predict(float dYawDeg, float dtSec);

    // GNSS step. measurementDeg is COG (true north, clockwise), 0..360.
    // noiseDeg is 1-sigma of that measurement, degrees.
    void correct(float measurementDeg, float noiseDeg);

    // Hard-align the heading estimate with a measurement without touching bias
    // or the convergence flag. Used while the estimate is still unaligned.
    void seedHeading(float measurementDeg);

    // Fused heading, 0..360.
    float heading() const;

    // Estimated drift of the raw heading, deg/s.
    float biasDegPerSec() const;

    // False until the first correct() - heading() equals the raw IMU yaw before it.
    bool hasUpdate() const;

    // Last innovation (measurement - predicted heading), deg. 0 before first use.
    float lastInnovationDeg() const;

    // Re-seed from raw yaw and drop all learning.
    void reset(float initialYawDeg);

    // Tuning knobs, exposed so they can be changed without rebuilding the model.
    // Process noise on heading and on bias.
    float qYawDeg2;
    float qBiasDeg2PerS2;
    // Floor for measurement noise when the caller does not provide one.
    float rDefaultDeg2;

private:
    static float wrap180(float deg);
    static float wrap360(float deg);

    float x_yaw;
    float x_bias;
    // Covariance, row-major 2x2.
    float p00, p01, p10, p11;

    bool updated;
    float innovation;
};

#endif // EKF_HEADING_H
