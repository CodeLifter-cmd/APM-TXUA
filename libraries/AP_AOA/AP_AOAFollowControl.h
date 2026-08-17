#pragma once

#include "AP_AOAPID.h"

class AP_AOAFollowControl
{
public:
    static constexpr uint8_t RESUME_VALID_FRAMES = 5;
    static constexpr uint32_t DATA_TIMEOUT_MS = 1000;

    struct Output {
        float throttle;
        float steering;
    };

    void configure(float dist_kp, float dist_ki, float dist_kd, float dist_imax,
                   float angle_kp, float angle_ki, float angle_kd, float angle_imax,
                   float target_dist, float max_speed, float steer_limit);
    bool accept_sample(float distance_m, float body_angle_deg, uint32_t timestamp_ms);
    void reject_sample();
    bool check_timeout(uint32_t now_ms, uint32_t timeout_ms);
    bool set_distance_window(float target_dist, float dist_hyst);
    bool set_steering_smoothing(float angle_deadzone_deg, float steering_rate_cd_s);
    void reset();
    const Output &output() const { return _output; }
    bool too_close() const { return _too_close; }
    uint8_t resume_frame_count() const { return _resume_frame_count; }

private:
    AP_AOAPID _dist_pid;
    AP_AOAPID _angle_pid;
    float _target_dist = 1.0f;
    float _dist_hyst = 0.3f;
    float _max_speed = 1.0f;
    float _steer_limit = 1.0f;
    float _angle_deadzone_deg = 0.0f;
    float _steering_rate_cd_s = 0.0f;
    uint32_t _last_valid_sample_ms = 0;
    uint32_t _last_control_sample_ms = 0;
    bool _have_valid_sample = false;
    bool _have_control_sample = false;
    bool _too_close = false;
    uint8_t _resume_frame_count = 0;
    bool _config_change_pending = false;
    Output _output{};

    void reset_pid_state();
};
