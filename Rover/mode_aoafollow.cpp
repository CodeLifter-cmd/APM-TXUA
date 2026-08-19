// mode_aoafollow.cpp
#include "Rover.h"
#include <AP_AOA/AP_AOADebugText.h>
const AP_Param::GroupInfo ModeAoafllow::var_info[] = {
    // PID参数
    AP_GROUPINFO("DIST_KP", 1, ModeAoafllow, _dist_kp, 0.8f),
    AP_GROUPINFO("DIST_KI", 2, ModeAoafllow, _dist_ki, 0.0f),
    AP_GROUPINFO("DIST_KD", 3, ModeAoafllow, _dist_kd, 0.0f),
    AP_GROUPINFO("ANGLE_KP", 4, ModeAoafllow, _angle_kp, 0.6f),
    AP_GROUPINFO("ANGLE_KI", 5, ModeAoafllow, _angle_ki, 0.0f),
    AP_GROUPINFO("ANGLE_KD", 6, ModeAoafllow, _angle_kd, 0.0f),
    // 运行参数
    AP_GROUPINFO("TARGET_DIST", 7, ModeAoafllow, _target_dist, 1.0f),
    AP_GROUPINFO("MAX_SPEED", 8, ModeAoafllow, _max_speed, 1.0f),
    AP_GROUPINFO("STEER_LIM", 9, ModeAoafllow, _steer_limit, 1.0f),
    AP_GROUPINFO("ANG_OFS", 10, ModeAoafllow, _angle_offset, 60.0f),

    // @Param: DIST_HYST
    // @DisplayName: AOA follow restart distance hysteresis
    // @Description: Distance added to TARGET_DIST before a stopped vehicle may resume
    // @Units: m
    // @Range: 0.05 2.0
    // @Increment: 0.05
    // @User: Standard
    AP_GROUPINFO("DIST_HYST", 12, ModeAoafllow, _dist_hyst, 0.3f),
    AP_GROUPINFO("FILT_TC", 13, ModeAoafllow, _filter_tc, 0.2f),
    AP_GROUPINFO("ANG_DZ", 14, ModeAoafllow, _angle_deadzone, 3.0f),
    AP_GROUPINFO("ANG_JUMP", 15, ModeAoafllow, _angle_jump, 60.0f),
    AP_GROUPINFO("STEER_RATE", 16, ModeAoafllow, _steering_rate, 18000.0f),

    // @Param: DEBUG
    // @DisplayName: AOA follow debug telemetry
    // @Description: Streams full-rate AOA DataFlash diagnostics and sends a 5Hz Mission Planner diagnostic snapshot
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("DEBUG", 19, ModeAoafllow, _debug, 0),
    AP_GROUPEND};

ModeAoafllow::ModeAoafllow() : Mode(), // 必须首先初始化基类
                               _was_armed(false),
                               _outputs_stopped(true),
                               _timeout_active(false),
                               _last_sample_ms(0),
                               _have_sample_time(false),
                               _throttle_out(0.0f),
                               _steering_out(0.0f)
{

    AP_Param::setup_object_defaults(this, var_info);
}

bool ModeAoafllow::_enter()
{
    // 初始化传感器
    aoa_sensor1.init(6);
    // aoa_sensor2.init(7);

    // multidist_sensor.init();    // 初始化多超声波传感器
    //写入PID参数
    _control.configure(_dist_kp.get(), _dist_ki.get(), _dist_kd.get(), 0.01f,
                       _angle_kp.get(), _angle_ki.get(), _angle_kd.get(), 0.01f,
                       _target_dist.get(), _max_speed.get(), _steer_limit.get());
    _control.set_steering_smoothing(_angle_deadzone.get(), _steering_rate.get());
    _kalman_filter.set_time_constant(_filter_tc.get());
    _kalman_filter.set_angle_jump_limit(_angle_jump.get());

    // 重置控制器状态
    reset_controllers();
    _debug_state = {};
    _was_armed = hal.util->get_soft_armed();
    stop_outputs();

    // 显示模式信息
    gcs().send_text(MAV_SEVERITY_INFO, "AOA Follow ENGAGED");
    if (_debug.get() != 0) {
        gcs().send_text(MAV_SEVERITY_INFO, "AOA debug: MP 5Hz and MAVLink LOG enabled");
    }
    return true;
}

void ModeAoafllow::update()
{
    // static float x_out = 0,y_out=0;
    const uint32_t now_ms = AP_HAL::millis();
    const bool armed = hal.util->get_soft_armed();
    if (!armed) {
        if (_was_armed) {
            reset_controllers();
        }
        _was_armed = false;
        stop_outputs();
        return;
    }
    if (!_was_armed) {
        _was_armed = true;
        aoa_sensor1.reset_session();
        reset_controllers();
        stop_outputs();
    }
    if (_control.set_distance_window(_target_dist.get(), _dist_hyst.get())) {
        stop_outputs();
        return;
    }
    const bool filter_changed = _kalman_filter.set_time_constant(_filter_tc.get()) |
                                _kalman_filter.set_angle_jump_limit(_angle_jump.get());
    const bool steering_config_changed =
        _control.set_steering_smoothing(_angle_deadzone.get(), _steering_rate.get());
    if (filter_changed || steering_config_changed) {
        _kalman_filter.reset();
        _last_sample_ms = 0;
        _have_sample_time = false;
        stop_outputs();
        return;
    }
    // // gcs().send_text(MAV_SEVERITY_INFO, "AOA Follow update start work");
    // // 1. 获取原始传感器数据
    float raw_dist1, raw_angle1;
    uint32_t sample_ms;
    uint32_t event_sequence;
    // float raw_dist2, raw_angle2;
    // float filtered_angle = 0;

    // static uint8_t t_cnt = 1; // 计算周期标志位

    // if (t_cnt * dt_ms >= 50)
    // {
    //     multidist_sensor.update();  //多超声波传感器采集更新程序
    //     t_cnt = 1;
    // }
    // else
    // {
    //     t_cnt++;
    // }


    aoa_sensor1.update();       //UWB跟随传感器跟随程序
    _drain_sensor_diagnostics();
    if (!aoa_sensor1.get_raw_data(raw_dist1, raw_angle1, &sample_ms, &event_sequence)) {
        _handle_data_loss(now_ms);
        return;
    }

    _debug_state.event_sequence = event_sequence;
    _debug_state.raw_dist = raw_dist1;
    _debug_state.raw_angle = raw_angle1;

    // gcs().send_text(MAV_SEVERITY_INFO, "传感器测量值:%f , %f", raw_dist1, raw_angle1);
    // // 2. 卡尔曼滤波更新
    const float sample_dt = _have_sample_time ? (sample_ms - _last_sample_ms) * 0.001f : 0.0f;
    const bool filter_ok = _kalman_filter.update(raw_angle1, raw_dist1, sample_dt);
    _debug_state.filtered_dist = _kalman_filter.get_distance();
    _debug_state.filtered_angle = _kalman_filter.get_angle();
    _debug_state.body_angle = _kalman_filter.get_body_angle(_angle_offset.get());
    _write_filter_diagnostics(event_sequence, sample_dt, raw_dist1, raw_angle1, filter_ok);
    if (!filter_ok) {
        gcs().send_text(MAV_SEVERITY_WARNING, "AOA invalid sample");
        _control.reject_sample();
        stop_outputs();
        _debug_state.actuator_log_pending = _debug.get() != 0;
        _handle_data_loss(now_ms);
        return;
    }
    _last_sample_ms = sample_ms;
    _have_sample_time = true;
    _timeout_active = false;

    // 3. 获取滤波状态
    const float filtered_dist1 = _debug_state.filtered_dist;
    // _kalman_filter.update(raw_dist2, raw_angle2);
    // const float filtered_dist2 = _kalman_filter.get_distance();
    // const float filtered_angle2 = _kalman_filter.get_angle();

    // //转换到笛卡尔坐标系

    // float filtered_dist = (filtered_dist1 + filtered_dist2) / 2;
    // if (filtered_dist2 > filtered_dist1)
    // {
    //     filtered_angle = filtered_angle1;
    // }
    // else
    // {
    //     filtered_angle = filtered_angle2;
    //     filtered_dist = -filtered_dist;
    // }
    

    const float body_angle_deg = _debug_state.body_angle;

    //底通滤波
    // x_out = x_out*0.5 + 0.5 * x;
    // y_out = y_out*0.5 + 0.5 * y;

    // gcs().send_named_float("x", x);
    // gcs().send_named_float("y", y);

    // 4. 距离锁存和PID控制
    const bool was_too_close = _control.too_close();
    const bool control_accepted = _control.accept_sample(filtered_dist1, body_angle_deg, sample_ms);
    _write_control_diagnostics(event_sequence);
    const auto &output = _control.output();
    _throttle_out = output.throttle;
    _steering_out = output.steering;
    _debug_state.actuator_log_pending = _debug.get() != 0;
    if (!control_accepted) {
        if (!was_too_close && _control.too_close()) {
            gcs().send_text(MAV_SEVERITY_WARNING, "AOA target distance reached");
        } else if (was_too_close && !_control.too_close()) {
            gcs().send_text(MAV_SEVERITY_INFO, "AOA target distance cleared");
        }
        stop_outputs();
        return;
    }
    const Vector2f control_out(output.throttle, output.steering);

    // 6. 执行器输出
    _set_actuators(control_out);
}

void ModeAoafllow::_handle_data_loss(uint32_t now_ms)
{
    if (_control.too_close()) {
        stop_outputs();
    }
    if (_control.check_timeout(now_ms, 1000U)) {
        if (!_timeout_active) {
            gcs().send_text(MAV_SEVERITY_WARNING, "AOA Data Timeout");
        }
        _timeout_active = true;
        _last_sample_ms = 0;
        _have_sample_time = false;
        stop_outputs();
    }
}

void ModeAoafllow::_set_actuators(const Vector2f &control)
{
    _throttle_out = control.x;
    _steering_out = control.y;
    rover.g2.motors.set_steering(control.y);
    rover.g2.motors.set_throttle(control.x);
    _outputs_stopped = is_zero(control.x) && is_zero(control.y);
}

void ModeAoafllow::_drain_sensor_diagnostics()
{
    AP_AOA_ALX_FIT::DiagnosticEvent event{};
    while (aoa_sensor1.get_diagnostic_event(event)) {
        if (_debug.get() == 0) {
            continue;
        }
        _debug_state.gating_rejects = event.gating_rejects;
        _debug_state.queue_drops = event.queue_drops;
        if (event.result != AP_AOA_ALX_FIT::GateResult::ACCEPTED &&
            event.result != AP_AOA_ALX_FIT::GateResult::HEARTBEAT) {
            _debug_state.last_reject_gate = uint8_t(event.result);
        }
#if HAL_LOGGING_ENABLED
        AP::logger().WriteStreaming(
            "AOAR", "TimeUS,SmpMS,Ev,Dist,Ang,Tag,Seq,Bat,Conf,Gate",
            "QIIffIHHBB",
            AP_HAL::micros64(),
            event.timestamp_ms,
            event.event_sequence,
            event.frame.distance_cm * 0.01f,
            float(event.frame.azimuth_deg),
            event.frame.tag_id,
            event.frame.seq_id,
            event.frame.batch_sn,
            event.frame.confidence,
            uint8_t(event.result));
        AP::logger().WriteStreaming(
            "AOAS", "TimeUS,Ev,DErr,GRej,GAcc,Hb,QDrop", "QIIIIII",
            AP_HAL::micros64(),
            event.event_sequence,
            event.decode_errors,
            event.gating_rejects,
            event.gating_accepts,
            event.heartbeats,
            event.queue_drops);
#endif
    }
}

void ModeAoafllow::_write_filter_diagnostics(uint32_t event_sequence, float sample_dt,
                                             float raw_dist, float raw_angle, bool filter_ok)
{
    if (_debug.get() == 0) {
        return;
    }
    _debug_state.sample_dt_ms = uint16_t(constrain_float(sample_dt * 1000.0f + 0.5f,
                                                         0.0f, 65535.0f));
    _debug_state.jump_candidate_count = _kalman_filter.jump_candidate_count();
#if HAL_LOGGING_ENABLED
    AP::logger().WriteStreaming(
        "AOAF", "TimeUS,Ev,Dt,RDist,RAng,FDist,FAng,Body,JCnt,OK",
        "QIffffffBB",
        AP_HAL::micros64(),
        event_sequence,
        sample_dt,
        raw_dist,
        raw_angle,
        _debug_state.filtered_dist,
        _debug_state.filtered_angle,
        _debug_state.body_angle,
        _kalman_filter.jump_candidate_count(),
        uint8_t(filter_ok));
#endif
}

void ModeAoafllow::_write_control_diagnostics(uint32_t event_sequence)
{
    if (_debug.get() == 0) {
        return;
    }
#if HAL_LOGGING_ENABLED
    const AP_AOAFollowControl::Diagnostics &diagnostics = _control.diagnostics();
    const AP_AOAPID::Info &dist_pid = _control.distance_pid_info();
    const AP_AOAPID::Info &angle_pid = _control.angle_pid_info();
    const AP_AOAFollowControl::Output &output = _control.output();
    AP::logger().WriteStreaming(
        "AOAC", "TimeUS,Ev,Dt,DErr,AErr,Thr,Str,Valid,Acc,Close,DZ,Resume",
        "QIfffffBBBBB",
        AP_HAL::micros64(),
        event_sequence,
        diagnostics.dt,
        diagnostics.distance_error,
        diagnostics.angle_error,
        output.throttle,
        output.steering,
        uint8_t(diagnostics.sample_valid),
        uint8_t(diagnostics.control_accepted),
        uint8_t(_control.too_close()),
        uint8_t(diagnostics.angle_deadzone),
        _control.resume_frame_count());
    AP::logger().WriteStreaming(
        "AOAP", "TimeUS,Ev,DP,DI,DD,DO,AP,AI,AD,AO", "QIffffffff",
        AP_HAL::micros64(),
        event_sequence,
        dist_pid.p,
        dist_pid.i,
        dist_pid.d,
        dist_pid.output,
        angle_pid.p,
        angle_pid.i,
        angle_pid.d,
        angle_pid.output);
#endif
}

void ModeAoafllow::write_actuator_log()
{
    if (_debug.get() == 0 || !_debug_state.actuator_log_pending) {
        return;
    }
    _debug_state.actuator_log_pending = false;

    const float left_output = SRV_Channels::get_output_scaled(SRV_Channel::k_throttleLeft);
    const float right_output = SRV_Channels::get_output_scaled(SRV_Channel::k_throttleRight);
    uint16_t left_pwm = 0;
    uint16_t right_pwm = 0;
    const bool left_pwm_valid = SRV_Channels::get_output_pwm(SRV_Channel::k_throttleLeft, left_pwm);
    const bool right_pwm_valid = SRV_Channels::get_output_pwm(SRV_Channel::k_throttleRight, right_pwm);
    uint8_t motor_flags = uint8_t(left_pwm_valid) | (uint8_t(right_pwm_valid) << 1U);
    motor_flags |= uint8_t(rover.g2.motors.limit.steer_left) << 2U;
    motor_flags |= uint8_t(rover.g2.motors.limit.steer_right) << 3U;
    motor_flags |= uint8_t(rover.g2.motors.limit.throttle_lower) << 4U;
    motor_flags |= uint8_t(rover.g2.motors.limit.throttle_upper) << 5U;

    const float speed = ahrs.groundspeed();
    const float yaw_rate = degrees(ahrs.get_gyro().z);
    const Vector3f &accel_ef = ahrs.get_accel_ef();
    float current = 0.0f;
    const bool current_valid = rover.battery.current_amps(current);
    const float voltage = rover.battery.voltage();

    _debug_state.left_output = left_output;
    _debug_state.right_output = right_output;
    _debug_state.speed = speed;
    _debug_state.yaw_rate = yaw_rate;

#if HAL_LOGGING_ENABLED
    AP::logger().WriteStreaming(
        "AOAM", "TimeUS,Ev,Thr,Str,Left,Right,LPWM,RPWM,Flags",
        "QIffffHHB",
        AP_HAL::micros64(),
        _debug_state.event_sequence,
        _throttle_out,
        _steering_out,
        left_output,
        right_output,
        left_pwm,
        right_pwm,
        motor_flags);
    AP::logger().WriteStreaming(
        "AOAV", "TimeUS,Ev,Spd,YawRt,Ax,Ay,Volt,Curr,CurrOK",
        "QIffffffB",
        AP_HAL::micros64(),
        _debug_state.event_sequence,
        speed,
        yaw_rate,
        accel_ef.x,
        accel_ef.y,
        voltage,
        current,
        uint8_t(current_valid));
#endif
    _send_debug_info(AP_HAL::millis());
}

void ModeAoafllow::_send_debug_info(uint32_t timestamp)
{
    if (timestamp - _debug_state.last_text_ms < 200U) {
        return;
    }
    _debug_state.last_text_ms = timestamp;
    const AP_AOADebugText::Snapshot snapshot{
        _debug_state.event_sequence,
        _debug_state.raw_dist,
        _debug_state.raw_angle,
        _debug_state.filtered_dist,
        _debug_state.filtered_angle,
        _debug_state.body_angle,
        _debug_state.sample_dt_ms,
        _debug_state.jump_candidate_count,
        _debug_state.last_reject_gate,
        _debug_state.gating_rejects,
        _debug_state.queue_drops,
        _throttle_out,
        _steering_out,
        _debug_state.left_output,
        _debug_state.right_output,
        _debug_state.speed,
        _debug_state.yaw_rate,
    };
    AP_AOADebugText::Lines lines{};
    AP_AOADebugText::format(snapshot, lines);
    for (const auto &line : lines) {
        gcs().send_text(MAV_SEVERITY_INFO, "%s", line);
    }
}

void ModeAoafllow::reset_controllers()
{
    _control.reset();
    _last_sample_ms = 0;
    _have_sample_time = false;
    // 重置积分项和微分项
    _throttle_out = 0.0f;
    _steering_out = 0.0f;
    // _last_debug_ms = 0;
    _kalman_filter.reset();
}

void ModeAoafllow::stop_outputs()
{
    if (!_outputs_stopped && _debug.get() != 0) {
        // Preserve asynchronous stop transitions such as timeout or disarm.
        _debug_state.actuator_log_pending = true;
    }
    _throttle_out = 0.0f;
    _steering_out = 0.0f;
    rover.g2.motors.set_throttle(0.0f);
    rover.g2.motors.set_steering(0.0f);
    _outputs_stopped = true;
}

// 模式退出处理
void ModeAoafllow::_exit()
{
    _debug_state.actuator_log_pending = false;
    reset_controllers();
    stop_outputs();
    gcs().send_text(MAV_SEVERITY_INFO, "AOA Follow DISENGAGED");
}

// // 在AP_Mission中注册模式
// const struct AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] = {
//     // ...
//     AP_GROUPINFO("MODE_AOA_FOLLOW", 23, GCS_MAVLINK_Parameters, mode_aoafollow, 0),
//     // ...
// };
