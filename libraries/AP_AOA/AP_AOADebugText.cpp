#include "AP_AOADebugText.h"

#include <AP_Math/AP_Math.h>

#include <stdio.h>

constexpr uint8_t AP_AOADebugText::LINE_COUNT;
constexpr uint8_t AP_AOADebugText::TEXT_LENGTH;

void AP_AOADebugText::format(const Snapshot &snapshot, Lines &lines)
{
    // Five digits are enough to correlate a field-test snapshot for more than
    // 40 minutes at 40Hz, while keeping every STATUSTEXT line below 50 bytes.
    const unsigned event = unsigned(snapshot.event_sequence % 100000U);
    const unsigned gating_rejects = unsigned(snapshot.gating_rejects % 100000U);
    const unsigned queue_drops = unsigned(snapshot.queue_drops % 100000U);

    snprintf(lines[0], TEXT_LENGTH,
             "A0 E%05u R%.1f/%.0f F%.1f/%.0f B%.0f D%u",
             event,
             constrain_float(snapshot.raw_distance_m, 0.0f, 99.9f),
             constrain_float(snapshot.raw_angle_deg, -180.0f, 180.0f),
             constrain_float(snapshot.filtered_distance_m, 0.0f, 99.9f),
             constrain_float(snapshot.filtered_angle_deg, -180.0f, 180.0f),
             constrain_float(snapshot.body_angle_deg, -180.0f, 180.0f),
             unsigned(snapshot.sample_dt_ms));
    snprintf(lines[1], TEXT_LENGTH,
             "A1 E%05u J%u G%u GR%u Q%u",
             event,
             unsigned(snapshot.jump_candidate_count),
             unsigned(snapshot.last_reject_gate),
             gating_rejects,
             queue_drops);
    snprintf(lines[2], TEXT_LENGTH,
             "A2 E%05u T%.0f S%.0f L%.0f R%.0f V%.1f Y%.1f",
             event,
             constrain_float(snapshot.throttle, 0.0f, 100.0f),
             constrain_float(snapshot.steering, -4500.0f, 4500.0f),
             constrain_float(snapshot.left_output, -1000.0f, 1000.0f),
             constrain_float(snapshot.right_output, -1000.0f, 1000.0f),
             constrain_float(snapshot.speed_m_s, 0.0f, 99.9f),
             constrain_float(snapshot.yaw_rate_deg_s, -999.9f, 999.9f));

    for (auto &line : lines) {
        line[TEXT_LENGTH - 1] = '\0';
    }
}
