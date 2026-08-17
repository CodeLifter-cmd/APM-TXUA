#include "AP_AOAPID.h"
#include <AP_Math/AP_Math.h>
AP_AOAPID::AP_AOAPID()
{
    // AP_Param::setup_object_defaults(this, var_info);
}

void AP_AOAPID::reset()
{
    // _kp.set(0);
    // _ki.set(0);
    // _kd.set(0);
    // _imax.set(0);
    _integrator = 0;
    _last_error = 0;
    _last_derivative = 0;
    _initialised = false;
}
void AP_AOAPID::set_gains(float kp, float ki, float kd, float imax)
{
    _kp.set(kp);
    _ki.set(ki);
    _kd.set(kd);
    _imax.set(imax);
}

float AP_AOAPID::get_pid(float error, float dt, float scaler)
{
    // _kp.set(0.8);
    // _ki.set(0.05);
    // _kd.set(0.2);
    // _imax.set(1);
    if (!isfinite(dt) || dt <= 0.0f)
    {
        return 0.0f;
    }

    float pterm = error * _kp;
    _integrator += error * _ki * dt;
    _integrator = constrain_float(_integrator, -_imax, _imax);

    float dterm = 0.0f;
    if (_initialised)
    {
        float derivative = (error - _last_error) / dt;
        derivative = 0.2f * derivative + 0.8f * _last_derivative;
        dterm = derivative * _kd;
        _last_derivative = derivative;
    }

    _last_error = error;
    _initialised = true;
    return (pterm + _integrator + dterm) * scaler;
}
