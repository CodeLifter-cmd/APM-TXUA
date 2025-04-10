#pragma once
#ifndef _FIRE_LED_H_
#define _FIRE_LED_H_
#include <AP_HAL/AP_HAL.h>
#define power_led_on hal.gpio->write(2, 1)
#define power_led_off hal.gpio->write(2, 0)
#define Alert_led_on hal.gpio->write(3, 1)
#define Alert_led_off hal.gpio->write(3, 0)
#define C1_on hal.gpio->write(8, 0)
#define C1_off hal.gpio->write(8, 1)
#define C2_on hal.gpio->write(9, 0)
#define C2_off hal.gpio->write(9, 1)

extern const AP_HAL::HAL &hal;

class Fire_LED
{
private:
    /* data */
public:
    Fire_LED(/* args */);
    void Fire_LED_Init();
    void Fire_Alert_LED();
    void Fire_Power_LED(float V, uint8_t DT_ms);
    void launch_motor();
    void stop_motor();
};

#endif