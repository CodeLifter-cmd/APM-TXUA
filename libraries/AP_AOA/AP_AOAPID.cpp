#include "AP_AOAPID.h"
#include <AP_Math/AP_Math.h>

// Preserve orign's derivative filter response: alpha=0.2 at 10Hz.
static constexpr float D_FILTER_ALPHA_10HZ = 0.2f;
static constexpr float D_FILTER_DT_10HZ = 0.1f;
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
    _info = {};
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
        _info = {};
        _info.error = error;
        _info.dt = dt;
        return 0.0f;
    }

    float pterm = error * _kp;
    _integrator += error * _ki * dt;
    _integrator = constrain_float(_integrator, -_imax, _imax);

    float dterm = 0.0f;
    if (_initialised)
    {
        float derivative = (error - _last_error) / dt;
        const float filter_dt = MIN(dt, D_FILTER_DT_10HZ);
        const float alpha = constrain_float(
            1.0f - powf(1.0f - D_FILTER_ALPHA_10HZ, filter_dt / D_FILTER_DT_10HZ),
            0.0f,
            1.0f);
        derivative = alpha * derivative + (1.0f - alpha) * _last_derivative;
        dterm = derivative * _kd;
        _last_derivative = derivative;
    }

    _last_error = error;
    _initialised = true;
    _info.error = error;
    _info.dt = dt;
    _info.p = pterm * scaler;
    _info.i = _integrator * scaler;
    _info.d = dterm * scaler;
    _info.output = _info.p + _info.i + _info.d;
    return _info.output;
}
