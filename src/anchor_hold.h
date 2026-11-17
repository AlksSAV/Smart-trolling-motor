#pragma once
#include "pid_controller.h"

struct ControlOutput {
    float servo_angle; // -vec_angle_max_deg ... +vec_angle_max_deg (условные градусы прошивки)
    float thrust;      // -100.0 ... 100.0 (отрицательное = реверс)
};

class AnchorHold {
private:
    PIDController thrust_pid;

    float target_lat = 0.0f;
    float target_lon = 0.0f;
    bool  is_active  = false;
    
    // Гистерезис для переключения направления тяги
    bool thrust_reversed = false;
    float reverse_enter_deg = 100.0f;  // Включаем реверс при >100° от носа
    float reverse_exit_deg  = 80.0f;   // Выключаем реверс при <80° от носа

    // Порог |тяги| в %, ниже которого руль не имеет смысла: винт ещё не даёт
    // упора, а отклонённый мотор уже разворачивает корпус реактивным моментом.
    float min_thrust_for_steer_percent = 5.0f;

    // Физический предел поворота мотора. Analog of ArduPilot MOT_VEC_ANGLEMAX
    // (AP_MotorsUGV::_vector_angle_max), clamped to 0..90 like they do. Requesting
    // more than the column can turn loses the thrust direction silently, so the
    // angle is clamped here, at the source, instead of at the output stage.
    //
    // Units are the firmware servo units of angleToServoUs(), NOT guaranteed
    // degrees: 90 = full 1000..2000 us swing. With the present +/-45 deg servo
    // that full swing is what the boat actually gets, so the default keeps the
    // behaviour unchanged. After the 180 deg servo swap this becomes the real
    // mechanical limit.
    float vec_angle_max_deg = 90.0f;
    
    // Параметры зоны удержания
    float loiter_radius_m = 2.0f;   // Радиус "мёртвой зоны" (метод удержания)
    float max_distance_m  = 10.0f;  // Расстояние для максимальной тяги
    
    // Замороженная тяга ESC при входе в HEADING HOLD
    int heading_hold_base_esc_us = 1500;

    static float normalizeAngleDiff(float target, float current);

public:
    AnchorHold();

    static float getDistance(float lat1, float lon1, float lat2, float lon2);
    static float getBearing (float lat1, float lon1, float lat2, float lon2);

    void setTarget(float lat, float lon);
    void activate();
    void deactivate();
    void setHeadingHoldBaseEsc(int us) { heading_hold_base_esc_us = us; }
    int getHeadingHoldBaseEsc() const { return heading_hold_base_esc_us; }
    bool isActive() const { return is_active; }
    bool isThrustReversed() const { return thrust_reversed; }

    float getTargetLat() const { return target_lat; }
    float getTargetLon() const { return target_lon; }

    ControlOutput update(float current_lat, float current_lon,
                         float current_heading_deg,
                         float current_speed_ms,
                         float dt);

    void setThrustTunings(float p, float i, float d) {
        thrust_pid.setTunings(p, i, d);
    }

    // Time constant of the derivative smoothing (seconds). 0 disables it.
    void setThrustDerivativeTau(float tau) {
        thrust_pid.setDerivativeTau(tau);
    }
    
    void setLoiterRadius(float radius) {
        loiter_radius_m = radius;
    }
    
    void setMaxDistance(float dist) {
        max_distance_m = dist;
    }
    
    void setReverseHysteresis(float enter_deg, float exit_deg) {
        reverse_enter_deg = enter_deg;
        reverse_exit_deg = exit_deg;
    }

    void setMinThrustForSteer(float percent) {
        min_thrust_for_steer_percent = fabsf(percent);
    }

    // Clamped to 0..90 like ArduPilot does for MOT_VEC_ANGLEMAX.
    void setVecAngleMax(float deg) {
        vec_angle_max_deg = constrainToVecRange(deg);
    }
    float getVecAngleMax() const { return vec_angle_max_deg; }

    static float constrainToVecRange(float deg) {
        if (deg < 0.0f)  return 0.0f;
        if (deg > 90.0f) return 90.0f;
        return deg;
    }
};
