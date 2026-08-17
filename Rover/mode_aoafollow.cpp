// mode_aoafollow.cpp
#include "Rover.h"
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
    _was_armed = hal.util->get_soft_armed();
    stop_outputs();

    // 显示模式信息
    gcs().send_text(MAV_SEVERITY_INFO, "AOA Follow ENGAGED");
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
    if (!aoa_sensor1.get_raw_data(raw_dist1, raw_angle1, &sample_ms)) {
        _handle_data_loss(now_ms);
        return;
    }

    // gcs().send_text(MAV_SEVERITY_INFO, "传感器测量值:%f , %f", raw_dist1, raw_angle1);
    // // 2. 卡尔曼滤波更新
    const float sample_dt = _have_sample_time ? (sample_ms - _last_sample_ms) * 0.001f : 0.0f;
    if (!_kalman_filter.update(raw_angle1, raw_dist1, sample_dt)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "AOA invalid sample");
        _control.reject_sample();
        stop_outputs();
        _handle_data_loss(now_ms);
        return;
    }
    _last_sample_ms = sample_ms;
    _have_sample_time = true;
    _timeout_active = false;

    // 3. 获取滤波状态
    const float filtered_dist1 = _kalman_filter.get_distance();
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
    

    const float body_angle_deg = _kalman_filter.get_body_angle(_angle_offset.get());

    //底通滤波
    // x_out = x_out*0.5 + 0.5 * x;
    // y_out = y_out*0.5 + 0.5 * y;

    // gcs().send_named_float("x", x);
    // gcs().send_named_float("y", y);

    // 4. 距离锁存和PID控制
    const bool was_too_close = _control.too_close();
    if (!_control.accept_sample(filtered_dist1, body_angle_deg, sample_ms)) {
        if (!was_too_close && _control.too_close()) {
            gcs().send_text(MAV_SEVERITY_WARNING, "AOA target distance reached");
        } else if (was_too_close && !_control.too_close()) {
            gcs().send_text(MAV_SEVERITY_INFO, "AOA target distance cleared");
        }
        stop_outputs();
        return;
    }
    const auto &output = _control.output();
    const Vector2f control_out(output.throttle, output.steering);

    // 6. 执行器输出
    _set_actuators(control_out);

    // 7. 调试输出
    // _send_debug_info(now_ms, filtered_dist1, filtered_angle1, control_out);
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
    rover.g2.motors.set_steering(control.y);
    rover.g2.motors.set_throttle(control.x);
    _outputs_stopped = is_zero(control.x) && is_zero(control.y);
}

void ModeAoafllow::_send_debug_info(uint32_t timestamp, float dist, float angle, const Vector2f &control)
{
    // static uint32_t _last_debug_ms;
#define AOA_DEBUG 0
#if AOA_DEBUG
        // 发送MAVLink调试信息（每200ms）
    if (timestamp - _last_debug_ms > 200)
    {
        _last_debug_ms = timestamp;
        mavlink_msg_aoa_debug_send(
            MAVLINK_COMM_0,
            timestamp,
            dist,
            angle,
            control.x,
            control.y,
            _kalman_filter.get_variance(0),
            _kalman_filter.get_variance(1));
    }
#endif
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
    _throttle_out = 0.0f;
    _steering_out = 0.0f;
    rover.g2.motors.set_throttle(0.0f);
    rover.g2.motors.set_steering(0.0f);
    _outputs_stopped = true;
}

// 模式退出处理
void ModeAoafllow::_exit()
{
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
