// libraries/AP_AOA/AP_AOAFilter.h
#pragma once

#include <stdint.h>

class AOAKalmanFilter
{
public:
    AOAKalmanFilter();
    void reset();
    void predict(float) {}
    bool update(float angle_deg, float distance_m, float dt = 0.0f);
    bool set_time_constant(float time_constant_s);
    bool set_angle_jump_limit(float angle_jump_deg);
    float get_angle() const { return _angle_deg; }
    float get_distance() const { return _distance_m; }
    float get_body_angle(float offset_deg) const;
    uint8_t jump_candidate_count() const { return _jump_candidate_count; }

private:
    float _angle_deg;
    float _distance_m;
    float _time_constant_s = 0.0f;
    float _angle_jump_deg = 0.0f;
    float _jump_candidate_deg = 0.0f;
    float _last_input_angle_deg = 0.0f;
    uint8_t _jump_candidate_count = 0;
    bool _initialised;
    bool _have_input_angle = false;
};
