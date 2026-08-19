#include "AP_AOAFilter.h"

#include <AP_Math/AP_Math.h>

// orign confirms a large jump for 3 frames at 10Hz (300ms). ALX receives
// AOA samples at 40Hz, so 12 frames preserve the same confirmation time.
static constexpr uint8_t JUMP_CONFIRM_FRAMES = 12;

AOAKalmanFilter::AOAKalmanFilter()
{
    reset();
}

void AOAKalmanFilter::reset()
{
    _angle_deg = 0.0f;
    _distance_m = 0.0f;
    _jump_candidate_deg = 0.0f;
    _jump_candidate_count = 0;
    _last_input_angle_deg = 0.0f;
    _have_input_angle = false;
    _initialised = false;
}

bool AOAKalmanFilter::set_angle_jump_limit(float angle_jump_deg)
{
    const float value = MAX(angle_jump_deg, 0.0f);
    if (is_equal(value, _angle_jump_deg)) { return false; }
    _angle_jump_deg = value;
    _jump_candidate_count = 0;
    return true;
}

bool AOAKalmanFilter::set_time_constant(float time_constant_s)
{
    const float value = MAX(time_constant_s, 0.0f);
    if (is_equal(value, _time_constant_s)) { return false; }
    _time_constant_s = value;
    return true;
}

bool AOAKalmanFilter::update(float angle_deg, float distance_m, float dt)
{
    if (!isfinite(angle_deg) || !isfinite(distance_m) || distance_m <= 0.0f) {
        return false;
    }
    const float wrapped_angle = wrap_180(angle_deg);
    if (_have_input_angle && _angle_jump_deg > 0.0f &&
        fabsf(wrap_180(wrapped_angle - _last_input_angle_deg)) > _angle_jump_deg) {
        if (_jump_candidate_count == 0 ||
            fabsf(wrap_180(wrapped_angle - _jump_candidate_deg)) > _angle_jump_deg) {
            _jump_candidate_deg = wrapped_angle;
            _jump_candidate_count = 1;
        } else {
            _jump_candidate_deg = wrapped_angle;
            _jump_candidate_count++;
        }
        _distance_m = distance_m;
        if (_jump_candidate_count < JUMP_CONFIRM_FRAMES) {
            return true;
        }
        _angle_deg = _jump_candidate_deg;
        _last_input_angle_deg = _jump_candidate_deg;
        _have_input_angle = true;
        _jump_candidate_count = 0;
        return true;
    }
    _jump_candidate_count = 0;
    _last_input_angle_deg = wrapped_angle;
    _have_input_angle = true;
    if (!_initialised || _time_constant_s <= 0.0f || !is_positive(dt)) {
        _angle_deg = wrapped_angle;
        _distance_m = distance_m;
        _initialised = true;
        return true;
    }
    const float alpha = constrain_float(dt / (_time_constant_s + dt), 0.0f, 1.0f);
    _angle_deg = wrap_180(_angle_deg + alpha * wrap_180(wrapped_angle - _angle_deg));
    _distance_m += alpha * (distance_m - _distance_m);
    return true;
}

float AOAKalmanFilter::get_body_angle(float offset_deg) const
{
    // ALX azimuth increases to the vehicle's left, while Rover steering is
    // positive to the right.  Invert the calibrated sensor angle here so all
    // downstream control and diagnostics use Rover's steering convention.
    return wrap_180(offset_deg - _angle_deg);
}
