#include "anchor_hold.h"
#include <math.h>

#define DEG_TO_RAD     0.017453292519943295f
#define RAD_TO_DEG     57.29577951308232f
#define EARTH_RADIUS   6371000.0f

AnchorHold::AnchorHold()
    // Значения по умолчанию: фактические задаёт setup() через setThrustTunings().
    // P=20.0: реакция на расстояние
    // I=0.1: компенсация постоянного сноса (течение/ветер)
    // D=+10.0: демпфирование. Знак "плюс" верен: PIDController() считает
    //      kd * d(error)/dt, а при приближении к цели error уменьшается,
    //      поэтому производная отрицательна и сама по себе гасит тягу.
    //      Отрицательный kd превратил бы демпфирование в раскачку.
    // deadzone=0.5 м: PIDController сравнивает её с error напрямую, а error
    //      здесь в метрах (distance - loiter_radius_m), а не в процентах.
    //      Прежние 5.0 означали молчание тяги до distance = 7 м из 10 м.
    //      0.5 м достаточно, чтобы ESC (своей 5% зоной) не "пел" у нуля.
    : thrust_pid(20.0f, 0.1f, 10.0f, -100.0f, 100.0f, 30.0f, 0.5f)
{
}

void AnchorHold::setTarget(float lat, float lon) {
    target_lat = lat;
    target_lon = lon;
    thrust_pid.reset();
    thrust_reversed = false;
}

void AnchorHold::activate() {
    is_active = true;
    thrust_pid.reset();
    thrust_reversed = false;
}

void AnchorHold::deactivate() {
    is_active = false;
    thrust_pid.reset();
    thrust_reversed = false;
}

float AnchorHold::getDistance(float lat1, float lon1, float lat2, float lon2) {
    float dLat = (lat2 - lat1) * DEG_TO_RAD;
    float dLon = (lon2 - lon1) * DEG_TO_RAD;
    float x = dLon * cosf((lat1 + lat2) * DEG_TO_RAD * 0.5f);
    float y = dLat;
    return sqrtf(x * x + y * y) * EARTH_RADIUS;
}

float AnchorHold::getBearing(float lat1, float lon1, float lat2, float lon2) {
    float dLon = (lon2 - lon1) * DEG_TO_RAD;
    float y = sinf(dLon) * cosf(lat2 * DEG_TO_RAD);
    float x = cosf(lat1 * DEG_TO_RAD) * sinf(lat2 * DEG_TO_RAD) -
              sinf(lat1 * DEG_TO_RAD) * cosf(lat2 * DEG_TO_RAD) * cosf(dLon);
    float bearing = atan2f(y, x) * RAD_TO_DEG;
    return fmodf(bearing + 360.0f, 360.0f);
}

float AnchorHold::normalizeAngleDiff(float target, float current) {
    float diff = target - current;
    while (diff > 180.0f)  diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

ControlOutput AnchorHold::update(float current_lat, float current_lon,
                                 float current_heading_deg,
                                 float current_speed_ms,
                                 float dt) {
    ControlOutput out = {0.0f, 0.0f};
    if (!is_active) return out;

    float distance = getDistance(current_lat, current_lon, target_lat, target_lon);
    
    // Мы уже в зоне удержания — отдыхаем
    if (distance <= loiter_radius_m) {
        thrust_pid.reset();
        thrust_reversed = false;
        return out; // servo=0, thrust=0
    }

    float bearing_to_target = getBearing(current_lat, current_lon, target_lat, target_lon);
    
    // Угол цели относительно носа лодки (-180 ... +180)
    float target_off_bow = normalizeAngleDiff(bearing_to_target, current_heading_deg);
    float abs_off_bow = fabsf(target_off_bow);

    // === Гистерезис для направления тяги ===
    // Включаем реверс только при >100°, выключаем при <80°
    if (!thrust_reversed && abs_off_bow > reverse_enter_deg) {
        thrust_reversed = true;
    } else if (thrust_reversed && abs_off_bow < reverse_exit_deg) {
        thrust_reversed = false;
    }

    // === Вычисление угла серво ===
    if (thrust_reversed) {
        // Цель сзади: поворачиваем серво на 180° от направления цели
        // и включаем реверс
        out.servo_angle = (target_off_bow > 0.0f) ? 
                          (target_off_bow - 180.0f) : 
                          (target_off_bow + 180.0f);
    } else {
        // Цель спереди: серво указывает прямо на цель
        out.servo_angle = target_off_bow;
    }

    // Ограничиваем угол серво ±90°
    if (out.servo_angle >  90.0f) out.servo_angle =  90.0f;
    if (out.servo_angle < -90.0f) out.servo_angle = -90.0f;

    // === Вычисление величины тяги через ПИД ===
    // Ошибка = текущее расстояние - радиус удержания
    float distance_error = distance - loiter_radius_m;
    
    // ПИД по расстоянию с демпфированием
    float raw_thrust = thrust_pid.compute(distance_error, dt);
    
    // Применяем направление (знак)
    out.thrust = thrust_reversed ? -raw_thrust : raw_thrust;

    return out;
}