// File: libraries/AP_Modbus/AP_MultiDistanceSensor.h
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/crc.h>
#include <GCS_MAVLink/GCS.h>

#define MODBUS_UART_NUM 4     // UART端口号(如UART2)
#define SENSOR_COUNT 6        // 最大传感器数量
#define PROCESSED_REG 0x0002  // 处理值寄存器地址
#define MODBUS_TIMEOUT_MS 300 // 单次通信超时

class AP_MultiDistanceSensor
{
public:
    AP_MultiDistanceSensor();
    void init();
    void update();
    bool get_distance(uint8_t sensor_idx, float &dist) const;
    // void update_sensor_task();
private:
    struct SensorData
    {
        uint8_t address;      // Modbus设备地址
        float distance;       // 当前距离值(m)
        uint32_t last_update; // 最后更新时间
        bool valid;           // 数据有效性标志
    };
    const AP_HAL::HAL &hal = AP_HAL::get_HAL();
    AP_HAL::UARTDriver *_uart4;
    // AP_MultiDistanceSensor(); // 单例模式构造函数


    SensorData sensors[SENSOR_COUNT];
    uint8_t current_sensor_idx;
};