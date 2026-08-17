// libraries/AP_PID/AP_AOAPID.h
#pragma once
#include <AP_Param/AP_Param.h>

class AP_AOAPID
{
public:
    AP_AOAPID();

    void reset();
    void set_gains(float kp, float ki, float kd, float imax);
    float get_pid(float error, float dt, float scaler = 1.0);

    // static const struct AP_Param::GroupInfo var_info[];

private:
    AP_Float _kp; 
    AP_Float _ki;
    AP_Float _kd;
    AP_Float _imax;
    float _integrator = 0;
    float _last_error = 0;
    float _last_derivative = 0;
    bool _initialised = false;
};

