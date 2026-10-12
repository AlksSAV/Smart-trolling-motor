#include "ekf_heading.h"

#include <math.h>

// Placeholder tuning values. Not measured: they only keep the filter stable
// until a log gives real numbers (see TODO).
static const float EKF_INIT_P_YAW_DEG2       = 4.0f;
static const float EKF_INIT_P_BIAS_DEG2_PER_S2 = 0.01f;

EkfHeading::EkfHeading()
    : qYawDeg2(0.001f),
      qBiasDeg2PerS2(1e-5f),
      rDefaultDeg2(25.0f),
      x_yaw(0.0f),
      x_bias(0.0f),
      p00(EKF_INIT_P_YAW_DEG2),
      p01(0.0f),
      p10(0.0f),
      p11(EKF_INIT_P_BIAS_DEG2_PER_S2),
      updated(false),
      innovation(0.0f) {}

float EkfHeading::wrap180(float deg) {
    float a = fmodf(deg, 360.0f);
    if (a > 180.0f)  a -= 360.0f;
    if (a < -180.0f) a += 360.0f;
    return a;
}

float EkfHeading::wrap360(float deg) {
    float a = fmodf(deg, 360.0f);
    if (a < 0.0f) a += 360.0f;
    return a;
}

void EkfHeading::begin(float initialYawDeg) {
    reset(initialYawDeg);
}

void EkfHeading::reset(float initialYawDeg) {
    x_yaw  = wrap360(initialYawDeg);
    x_bias = 0.0f;
    p00    = EKF_INIT_P_YAW_DEG2;
    p01    = 0.0f;
    p10    = 0.0f;
    p11    = EKF_INIT_P_BIAS_DEG2_PER_S2;
    updated    = false;
    innovation = 0.0f;
}

// Linear model over dt:
//   yaw'  = yaw + (dYaw - bias * dt)
//   bias' = bias
// F = [[1, -dt], [0, 1]]
void EkfHeading::predict(float dYawDeg, float dtSec) {
    if (dtSec <= 0.0f) return;

    x_yaw  = wrap360(x_yaw + (dYawDeg - x_bias * dtSec));
    // bias unchanged by the model.

    float n00 = p00 - dtSec * (p10 + p01) + dtSec * dtSec * p11;
    float n01 = p01 - dtSec * p11;
    float n10 = p10 - dtSec * p11;
    float n11 = p11;

    // Q is a noise rate: scale by dt so the tuning is independent of the loop rate.
    p00 = n00 + qYawDeg2 * dtSec;
    p01 = n01;
    p10 = n10;
    p11 = n11 + qBiasDeg2PerS2 * dtSec;
}

// Measurement model: z = yaw + noise, H = [1, 0].
void EkfHeading::correct(float measurementDeg, float noiseDeg) {
    float r = (noiseDeg > 0.0f) ? (noiseDeg * noiseDeg) : rDefaultDeg2;

    float s = p00 + r;
    if (s <= 0.0f) return;

    float y = wrap180(measurementDeg - x_yaw);
    float k0 = p00 / s;
    float k1 = p10 / s;

    x_yaw  = wrap360(x_yaw + k0 * y);
    x_bias = x_bias + k1 * y;

    // P = (I - K H) P
    float n00 = p00 * (1.0f - k0);
    float n01 = p01 * (1.0f - k0);
    float n10 = p10 - k1 * p00;
    float n11 = p11 - k1 * p01;

    // Symmetrize: the update above is not symmetric-preserving in float arithmetic.
    float off = 0.5f * (n01 + n10);
    p00 = n00;
    p11 = n11;
    p01 = off;
    p10 = off;

    innovation = y;
    updated = true;
}

// Hard-align yaw with the measurement. Does not touch bias, covariance, or the
// convergence flag. The caller uses this for the initial GNSS alignment (removes
// the constant offset between the IMU heading and true north — mounting +
// declination + mag bias). After seeding the next correct() that falls within
// the innovation gate will mark the filter converged.
void EkfHeading::seedHeading(float measurementDeg) {
    x_yaw = wrap360(measurementDeg);
    innovation = 0.0f;
}

float EkfHeading::heading() const {
    return x_yaw;
}

float EkfHeading::biasDegPerSec() const {
    return x_bias;
}

bool EkfHeading::hasUpdate() const {
    return updated;
}

float EkfHeading::lastInnovationDeg() const {
    return innovation;
}
