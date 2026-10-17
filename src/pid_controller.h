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
    float filtered_derivative = 0.0f;
    // Постоянная времени ФИНЧ (экспоненциального сглаживателя) для
    // производной. Дифференцирование разности GNSS-фиксов на dt = 0.05
    // усиливает дрожание позиции в 20 раз (0.1 м шума -> 2 м/с ->
    // +/-20% тяги при kd = 10), поэтому сырое значение в расчёт не идёт.
    float deriv_tau = 0.15f;
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

        // Deadzone: output is suppressed, but prev_error still tracks the
        // input. Zeroing it here would make the first derivative after the
        // deadzone (error - 0)/dt and kick the output.
        if (fabsf(error) < deadzone) {
            prev_error = error;
            filtered_derivative = 0.0f;
            return 0.0f;
        }

        integral += error * dt;
        if (integral >  integral_max) integral =  integral_max;
        if (integral < -integral_max) integral = -integral_max;

        float raw_derivative = (error - prev_error) / dt;
        // alpha = dt / (tau + dt): коэффициент не зависит от частоты вызова,
        // поэтому сглаживание остаётся прежним при другом dt.
        float alpha = dt / (deriv_tau + dt);
        filtered_derivative += alpha * (raw_derivative - filtered_derivative);

        float output = (kp * error) + (ki * integral) + (kd * filtered_derivative);

        if (output > output_max) output = output_max;
        if (output < output_min) output = output_min;

        prev_error = error;
        return output;
    }

    void reset() {
        integral   = 0.0f;
        prev_error = 0.0f;
        filtered_derivative = 0.0f;
    }

    void setTunings(float p, float i, float d) {
        kp = p; ki = i; kd = d;
    }

    // tau = 0 отключает сглаживание (сырая производная).
    void setDerivativeTau(float tau) {
        deriv_tau = (tau > 0.0f) ? tau : 0.0f;
    }
};