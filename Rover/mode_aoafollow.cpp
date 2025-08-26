// mode_aoafollow.cpp
#include "Rover.h"
const AP_Param::GroupInfo ModeAoafllow::var_info[] = {
    // PID参数
    AP_GROUPINFO("DIST_KP", 1, ModeAoafllow, _dist_kp, 0.8f),
    AP_GROUPINFO("DIST_KI", 2, ModeAoafllow, _dist_ki, 0.05f),
    AP_GROUPINFO("DIST_KD", 3, ModeAoafllow, _dist_kd, 0.2f),
    AP_GROUPINFO("ANGLE_KP", 4, ModeAoafllow, _angle_kp, 0.6f),
    AP_GROUPINFO("ANGLE_KI", 5, ModeAoafllow, _angle_ki, 0.1f),
    AP_GROUPINFO("ANGLE_KD", 6, ModeAoafllow, _angle_kd, 0.15f),
    // 运行参数
    AP_GROUPINFO("TARGET_DIST", 7, ModeAoafllow, _target_dist, 1.0f),
    AP_GROUPINFO("MAX_SPEED", 8, ModeAoafllow, _max_speed, 1.0f),
    AP_GROUPINFO("STEER_LIM", 9, ModeAoafllow, _steer_limit, 1.0f),
    AP_GROUPEND};

ModeAoafllow::ModeAoafllow() : Mode(), // 必须首先初始化基类
                               _last_update_ms(0),
                               _data_timeout_ms(0),
                               _throttle_out(0.0f),
                               _steering_out(0.0f),
                               _emergency_stop(false)
{

    AP_Param::setup_object_defaults(this, var_info);
}

bool ModeAoafllow::_enter()
{
    // 初始化传感器
    aoa_sensor1.init(6);
    aoa_sensor2.init(7);

    // multidist_sensor.init();    // 初始化多超声波传感器
    //写入PID参数
    _dist_pid.set_gains(_dist_kp.get(), _dist_ki.get(), _dist_kd.get(), 0.01);
    _angle_pid.set_gains(_angle_kp.get(), _angle_ki.get(), _angle_kd.get(), 0.01);

    // 重置控制器状态
    reset_controllers();

    // 显示模式信息
    gcs().send_text(MAV_SEVERITY_INFO, "AOA Follow ENGAGED");
    return true;
}

void ModeAoafllow::update()
{
    // static float x_out = 0,y_out=0;
    // const uint32_t now_ms = AP_HAL::millis();
    // // const float dt_ms = (now_ms - _last_update_ms);
    // const float dt = (now_ms - _last_update_ms) * 0.001f;
    // _last_update_ms = now_ms;
    // // gcs().send_text(MAV_SEVERITY_INFO, "AOA Follow update start work");
    // // 1. 获取原始传感器数据
    // float raw_dist1, raw_angle1;
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
    aoa_sensor2.update();
    // if (!aoa_sensor1.get_raw_data(raw_dist1, raw_angle1) ||
    //     !aoa_sensor2.get_raw_data(raw_dist2, raw_angle2))
    // {
    //     _handle_data_loss(dt);
    //     return;
    // }
    // gcs().send_named_float("dist1",raw_dist1);
    // gcs().send_named_float("raw_angle1", raw_angle1);

    // gcs().send_named_float("dist2", raw_dist2);
    // gcs().send_named_float("raw_angle2", raw_angle2);

    // // gcs().send_text(MAV_SEVERITY_INFO, "传感器测量值:%f , %f", raw_dist, raw_angle);
    // // 2. 卡尔曼滤波更新
    // _kalman_filter.predict(dt);
    // _kalman_filter.update(raw_dist1, raw_angle1);
    
    // _data_timeout_ms = now_ms;

    // // 3. 获取滤波状态
    // const float filtered_dist1 = _kalman_filter.get_distance();
    // const float filtered_angle1 = _kalman_filter.get_angle();

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
    

    // float y = filtered_dist * cosf(filtered_angle * 0.01745f);
    // float x = filtered_angle;
    
    // if (abs(y) > 20)
    // {
    //     y = 20 * (y/abs(y));
    // }

    // if (abs(x) > 60)
    // {
    //     x = 60 * (x / abs(x));
    // }

    // //底通滤波
    // x_out = x_out*0.5 + 0.5 * x;
    // y_out = y_out*0.5 + 0.5 * y;

    // gcs().send_named_float("x", x_out);
    // gcs().send_named_float("y", y_out);

    // // 4. 安全监测
    // if (!_safety_check(filtered_dist))
    // {
    //     return;
    // }

    // // 5. PID控制计算
    // Vector2f control_out = _calculate_control(y, x, dt);

    // // 6. 执行器输出
    // _set_actuators(control_out);

    // // 7. 调试输出
    // _send_debug_info(now_ms, filtered_dist, filtered_angle, control_out);
}

void ModeAoafllow::_handle_data_loss(float dt)
{
    // 数据超时处理（超过1秒无数据）
    if (AP_HAL::millis() - _data_timeout_ms > 1000)
    {
        gcs().send_text(MAV_SEVERITY_WARNING, "AOA Data Timeout!");
        // 缓降速处理
        _throttle_out *= 0.8f;
        _steering_out *= 0.8f;
        _set_actuators(Vector2f(_throttle_out, _steering_out));
        // rover.set_mode(rover.mode_hold, ModeReason::FAILSAFE);
        // gcs().send_text(MAV_SEVERITY_WARNING, "AOA Data Timeout!");
        return;
    }

}

bool ModeAoafllow::_safety_check(float current_dist)
{
    float target_dist = _target_dist.get();
    // static bool mul_flag_stop = false; // 多超声波避障传感器
    // uint8_t stop_cnt = 0;              // 判断多个传感器是否达到停止
    // float dist;
    // for (uint8_t i = 0; i < 6; i++)
    // {
    //     if (multidist_sensor.get_distance(i, dist))
    //     {
    //         gcs().send_text(MAV_SEVERITY_INFO, "Sensor%d: %.2fm", i, dist);
    //     }

    //     if (dist < 2000.0f)
    //     {
    //         mul_flag_stop = true;
    //         stop_cnt++;
    //     }
    //     /* code */
    // }

    // if (stop_cnt == 0)
    // {
    //     mul_flag_stop = false;
    // }


    // 紧急制动检查
    // if ((current_dist < _target_dist) || (mul_flag_stop))
    if (current_dist < target_dist)
    {
        _emergency_stop = true;
        rover.g2.motors.set_throttle(0);
        // rover.g2.motors.set_steering(0);

        gcs().send_text(MAV_SEVERITY_EMERGENCY, "EMERGENCY STOP!");
        return false;
    }

    // 重置急停状态
    // if (_emergency_stop && current_dist > target_dist + 0.5f && mul_flag_stop == false)
    if (_emergency_stop && current_dist > (target_dist + 0.5f))
    {
        _emergency_stop = false;
        reset_controllers();
    }
    return true;
}

Vector2f ModeAoafllow::_calculate_control(float dist, float angle, float dt)
{
    float target_dist = _target_dist.get();
    // 距离控制
    float dist_error = target_dist - dist;
    _throttle_out = _dist_pid.get_pid(dist_error, dt, 1.0f / _max_speed);
    // _throttle_out = 0;
    // 角度控制
    _steering_out = _angle_pid.get_pid(angle/10.0f, dt, 1.0f / _steer_limit);
    // _steering_out = 0;
    // 输出限幅
    _throttle_out = constrain_float(_throttle_out, -1.0f, 1.0f);
    _steering_out = constrain_float(_steering_out, -1.0f, 1.0f);
    // gcs().send_text(MAV_SEVERITY_INFO, "dist_error:%f,_throttle_out:%f,_steering_out:%f", dist_error, _throttle_out, _steering_out);

    return Vector2f(_throttle_out, _steering_out);
}

void ModeAoafllow::_set_actuators(const Vector2f &control)
{
    if (_emergency_stop)
    {
        rover.g2.motors.set_throttle(0);
        // rover.g2.motors.set_steering(0);
        return;
    }
    // gcs().send_named_float("set_steering：", (control.y * _steer_limit) * 4500);
    // gcs().send_named_float("set_throttle：", (control.x * _max_speed) * 100);
    // 设置转向和油门
    if (abs(control.y) > 0.06)
    {
        int8_t i = control.y/abs(control.y);
        rover.g2.motors.set_steering(-(control.y * _steer_limit) * 4000 + 450*i);
        /* code */
    }
    else
    {
        rover.g2.motors.set_steering(0);
    }

    if (abs(control.x) > 0.02)
    {
        int8_t i = -control.x / abs(control.x);
        rover.g2.motors.set_throttle((control.x * _max_speed) * 80 + i*10);
        /* code */
    }
    else
    {
        rover.g2.motors.set_throttle(0);
    }
    
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
    _dist_pid.reset();
    _angle_pid.reset();
    // 重置积分项和微分项
    _throttle_out = 0.0f;
    _steering_out = 0.0f;
    // _last_debug_ms = 0;
    _kalman_filter.reset();
}

// 模式退出处理
void ModeAoafllow::_exit()
{
    rover.g2.motors.set_throttle(0);
    rover.g2.motors.set_steering(0);
    gcs().send_text(MAV_SEVERITY_INFO, "AOA Follow DISENGAGED");
}

// // 在AP_Mission中注册模式
// const struct AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] = {
//     // ...
//     AP_GROUPINFO("MODE_AOA_FOLLOW", 23, GCS_MAVLINK_Parameters, mode_aoafollow, 0),
//     // ...
// };
