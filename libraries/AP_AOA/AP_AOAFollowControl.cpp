#include "AP_AOAFollowControl.h"

#include <AP_Math/AP_Math.h>

void AP_AOAFollowControl::configure(float dist_kp, float dist_ki, float dist_kd, float dist_imax,
                                    float angle_kp, float angle_ki, float angle_kd, float angle_imax,
                                    float target_dist, float max_speed, float steer_limit)
{
    _dist_pid.set_gains(dist_kp, dist_ki, dist_kd, dist_imax);
    _angle_pid.set_gains(angle_kp, angle_ki, angle_kd, angle_imax);
    _target_dist = MAX(target_dist, 0.05f);
    _max_speed = max_speed;
    _steer_limit = steer_limit;
    reset();
}

bool AP_AOAFollowControl::accept_sample(float distance_m, float body_angle_deg, uint32_t timestamp_ms)
{
    _diagnostics = {};
    _diagnostics.distance_m = distance_m;
    _diagnostics.body_angle_deg = body_angle_deg;
    _diagnostics.distance_error = distance_m - _target_dist;
    _diagnostics.angle_error = body_angle_deg;
    if (!isfinite(distance_m) || !isfinite(body_angle_deg) || distance_m <= 0.0f) {
        return false;
    }
    _diagnostics.sample_valid = true;

    if (_have_valid_sample && timestamp_ms - _last_valid_sample_ms > DATA_TIMEOUT_MS) {
        reset_pid_state();
        _resume_frame_count = 0;
    }
    _last_valid_sample_ms = timestamp_ms;
    _have_valid_sample = true;

    if (!_too_close && distance_m <= _target_dist) {
        _too_close = true;
        _resume_frame_count = 0;
        _config_change_pending = false;
        reset_pid_state();
        return false;
    }

    if (_too_close) {
        reset_pid_state();
        if (distance_m < _target_dist + _dist_hyst) {
            _resume_frame_count = 0;
            _config_change_pending = false;
            return false;
        }
        if (_resume_frame_count < RESUME_VALID_FRAMES) {
            _resume_frame_count++;
        }
        _config_change_pending = false;
        if (_resume_frame_count < RESUME_VALID_FRAMES) {
            return false;
        }
        _too_close = false;
        _resume_frame_count = 0;
        return false;
    }

    if (_config_change_pending) {
        _config_change_pending = false;
        reset_pid_state();
        return false;
    }

    float dt = 0.0f;
    if (_have_control_sample) {
        dt = (timestamp_ms - _last_control_sample_ms) * 0.001f;
        if (!isfinite(dt) || dt <= 0.0f || dt > 1.0f) {
            reset_pid_state();
            dt = 0.0f;
        }
    }
    _diagnostics.dt = dt;

    _last_control_sample_ms = timestamp_ms;
    _have_control_sample = true;
    if (dt <= 0.0f) {
        // Initialise PID history without accumulating or differentiating time.
        dt = 1.0e-6f;
    }

    const float dist_error = distance_m - _target_dist;
    _output.throttle = constrain_float(_dist_pid.get_pid(dist_error, dt) * 100.0f,
                                       0.0f, _max_speed * 100.0f);
    const bool in_angle_deadzone = fabsf(body_angle_deg) <= _angle_deadzone_deg;
    _diagnostics.angle_deadzone = in_angle_deadzone;
    if (in_angle_deadzone) {
        _angle_pid.reset();
        _output.steering = 0.0f;
    } else {
        const float steering_target = constrain_float(_angle_pid.get_pid(body_angle_deg, dt) * 100.0f,
                                                      -_steer_limit * 4500.0f, _steer_limit * 4500.0f);
        if (_steering_rate_cd_s > 0.0f) {
        const float max_change = _steering_rate_cd_s * dt;
        _output.steering += constrain_float(steering_target - _output.steering, -max_change, max_change);
        } else {
            _output.steering = steering_target;
        }
    }
    _diagnostics.control_accepted = true;
    return true;
}

void AP_AOAFollowControl::reject_sample()
{
    _diagnostics = {};
    reset_pid_state();
    _resume_frame_count = 0;
}

bool AP_AOAFollowControl::check_timeout(uint32_t now_ms, uint32_t timeout_ms)
{
    if (!_have_valid_sample || now_ms - _last_valid_sample_ms <= timeout_ms) {
        return false;
    }
    reset_pid_state();
    _resume_frame_count = 0;
    _have_valid_sample = false;
    return true;
}

bool AP_AOAFollowControl::set_distance_window(float target_dist, float dist_hyst)
{
    const float new_target_dist = MAX(target_dist, 0.05f);
    const float new_dist_hyst = constrain_float(dist_hyst, 0.05f, 2.0f);
    if (is_equal(new_target_dist, _target_dist) && is_equal(new_dist_hyst, _dist_hyst)) {
        return false;
    }
    _target_dist = new_target_dist;
    _dist_hyst = new_dist_hyst;
    _resume_frame_count = 0;
    _config_change_pending = true;
    reset_pid_state();
    return true;
}

bool AP_AOAFollowControl::set_steering_smoothing(float angle_deadzone_deg, float steering_rate_cd_s)
{
    const float new_deadzone = constrain_float(angle_deadzone_deg, 0.0f, 45.0f);
    const float new_rate = MAX(steering_rate_cd_s, 0.0f);
    if (is_equal(new_deadzone, _angle_deadzone_deg) && is_equal(new_rate, _steering_rate_cd_s)) {
        return false;
    }
    _angle_deadzone_deg = new_deadzone;
    _steering_rate_cd_s = new_rate;
    reset_pid_state();
    return true;
}

void AP_AOAFollowControl::reset_pid_state()
{
    _dist_pid.reset();
    _angle_pid.reset();
    _last_control_sample_ms = 0;
    _have_control_sample = false;
    _output = {};
}

void AP_AOAFollowControl::reset()
{
    reset_pid_state();
    _last_valid_sample_ms = 0;
    _have_valid_sample = false;
    _too_close = false;
    _resume_frame_count = 0;
    _config_change_pending = false;
    _diagnostics = {};
}
