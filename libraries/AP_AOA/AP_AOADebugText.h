#pragma once

#include <stdint.h>

class AP_AOADebugText
{
public:
    static constexpr uint8_t LINE_COUNT = 3;
    static constexpr uint8_t TEXT_LENGTH = 50;
    using Lines = char[LINE_COUNT][TEXT_LENGTH];

    struct Snapshot {
        uint32_t event_sequence;
        float raw_distance_m;
        float raw_angle_deg;
        float filtered_distance_m;
        float filtered_angle_deg;
        float body_angle_deg;
        uint16_t sample_dt_ms;
        uint8_t jump_candidate_count;
        uint8_t last_reject_gate;
        uint32_t gating_rejects;
        uint32_t queue_drops;
        float throttle;
        float steering;
        float left_output;
        float right_output;
        float speed_m_s;
        float yaw_rate_deg_s;
    };

    static void format(const Snapshot &snapshot, Lines &lines);
};
