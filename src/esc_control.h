#pragma once
#include <Arduino.h>

class ESCController {
private:
    int pwm_min     = 1000; // Макс реверс
    int pwm_neutral = 1500; // Стоп
    int pwm_max     = 2000; // Макс вперед

    int   current_pwm       = 1500;
    float ramp_rate         = 100.0f; // единиц PWM в секунду
    float deadzone_percent  = 5.0f;
    unsigned long last_update_time = 0;

public:
    ESCController() {
        last_update_time = millis();
    }

    // Сырое преобразование процентов тяги в PWM
    int getEscPwm(float thrust_percent) {
        if (fabsf(thrust_percent) < deadzone_percent) {
            thrust_percent = 0.0f;
        }

        int target_pwm;
        if (thrust_percent > 0.0f) {
            target_pwm = map((long)thrust_percent,
                             (long)deadzone_percent, 100L,
                             pwm_neutral + 10, pwm_max);
        } else if (thrust_percent < 0.0f) {
            target_pwm = map((long)thrust_percent,
                             -(long)deadzone_percent, -100L,
                             pwm_neutral - 10, pwm_min);
        } else {
            target_pwm = pwm_neutral;
        }

        if (target_pwm > pwm_max) target_pwm = pwm_max;
        if (target_pwm < pwm_min) target_pwm = pwm_min;
        return target_pwm;
    }

    // Преобразование с плавным нарастанием (защита ESC)
    int getSmoothedPwm(float thrust_percent) {
        int target_pwm = getEscPwm(thrust_percent);

        unsigned long now = millis();
        float dt = (now - last_update_time) / 1000.0f;
        if (dt <= 0.0f) dt = 0.001f;
        last_update_time = now;

        float max_change = ramp_rate * dt;
        int pwm_diff = target_pwm - current_pwm;

        if (abs(pwm_diff) > (int)max_change) {
            current_pwm += (pwm_diff > 0) ? (int)max_change : -(int)max_change;
        } else {
            current_pwm = target_pwm;
        }
        return current_pwm;
    }

    void setDirectPwm(int pwm) { current_pwm = pwm; }
    void setRampRate(float rate) { ramp_rate = rate; }
    void setDeadzone(float dz)   { deadzone_percent = dz; }
    int  getCurrentPwm() const   { return current_pwm; }
};