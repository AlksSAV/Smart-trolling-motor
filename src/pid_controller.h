#pragma once
#include <math.h>

// Нормализация угла в диапазон [-180, +180]
inline float normalizeAngle(float angle) {
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

class PIDController {
private:
    float kp, ki, kd;
    float integral = 0.0f;
    float prev_error = 0.0f;
    float output_min, output_max;
    float integral_max;
    float deadzone;

public:
    PIDController(float kp, float ki, float kd,
                  float out_min, float out_max,
                  float max_i = 100.0f, float dz = 0.0f)
        : kp(kp), ki(ki), kd(kd),
          output_min(out_min), output_max(out_max),
          integral_max(max_i), deadzone(dz) {}

    float compute(float error, float dt) {
        if (dt <= 0.0f) return 0.0f;

        // Мертвая зона
        if (fabsf(error) < deadzone) {
            prev_error = 0.0f;
            return 0.0f;
        }

        integral += error * dt;
        if (integral >  integral_max) integral =  integral_max;
        if (integral < -integral_max) integral = -integral_max;

        float derivative = (error - prev_error) / dt;
        float output = (kp * error) + (ki * integral) + (kd * derivative);

        if (output > output_max) output = output_max;
        if (output < output_min) output = output_min;

        prev_error = error;
        return output;
    }

    void reset() {
        integral   = 0.0f;
        prev_error = 0.0f;
    }

    void setTunings(float p, float i, float d) {
        kp = p; ki = i; kd = d;
    }
};