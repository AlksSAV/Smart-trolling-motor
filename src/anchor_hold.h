#pragma once
#include "pid_controller.h"

struct ControlOutput {
    float servo_angle; // -90.0 ... 90.0 (градусы от диаметральной плоскости)
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
};
