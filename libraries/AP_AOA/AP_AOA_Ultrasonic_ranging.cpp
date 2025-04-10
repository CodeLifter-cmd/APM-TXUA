// File: libraries/AP_Modbus/AP_MultiDistanceSensor.cpp
#include "AP_AOA_Ultrasonic_ranging.h"

AP_MultiDistanceSensor::AP_MultiDistanceSensor() : current_sensor_idx(0)
{
    ;
}

void AP_MultiDistanceSensor::init()
{
    // 初始化传感器地址(0x01~0x06)
    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    {
        sensors[i].address = 0x01 + i;
        sensors[i].distance = 6666.0f;
        sensors[i].last_update = 0;
        sensors[i].valid = false;
    }

    // 配置UART参数
    _uart4 = hal.serial(4);
    _uart4->begin(115200, 256, 256);
    _uart4->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
}

void AP_MultiDistanceSensor::update()
{
    SensorData sensor = sensors[current_sensor_idx];

    // 构造请求帧
    uint8_t request[8] = {
        sensor.address,
        0x03,
        (PROCESSED_REG >> 8),
        (PROCESSED_REG & 0xFF),
        0x00, 0x01,
        0x00, 0x00};

    // 添加CRC
    uint16_t crc = calc_crc_modbus(request, sizeof(request) - 2);
    request[6] = crc & 0xFF;
    request[7] = (crc >> 8) & 0xFF;
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "start collect:%d", current_sensor_idx);
    // 发送请求
    _uart4->write(request, 8);
    // 接收响应
    uint8_t response[255] = {0};
    // uint32_t start = AP_HAL::millis();
    uint8_t idx = 0;

    // while ((AP_HAL::millis() - start) < MODBUS_TIMEOUT_MS)
    // {
    uint8_t rec_num = _uart4->available();
    
    while (rec_num > 0)
    {
        rec_num--; 
        response[idx++] = _uart4->read();

    }
    // }
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%#X,%#X,%#X,%#X,%#X,%#X,%#X,", response[0], response[1], response[2], response[3], response[4], response[5], response[6]);
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "crc:%#X", calc_crc_modbus(response, 5));
    // 解析响应
    if (idx >= 5 &&
        response[1] == 0x03 &&
        response[2] == 0x02)
    {
        if (calc_crc_modbus(response, 5) == ((response[6] << 8) | response[5]))
        {
            uint16_t raw_val = (response[3] << 8) | response[4];
            sensor.distance = raw_val;
            sensor.valid = true;
            sensor.last_update = AP_HAL::millis();
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "S%d: %.2fm OK",
                          response[0], sensor.distance*0.001f);
        }
        else
        {
            sensor.valid = false;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "S%d CRC Error",
                          response[0]);
        }
    }

    current_sensor_idx = (current_sensor_idx + 1) % SENSOR_COUNT;
}

bool AP_MultiDistanceSensor::get_distance(uint8_t sensor_idx, float &dist) const
{
    if (sensor_idx >= SENSOR_COUNT)
        return false;
    dist = sensors[sensor_idx].distance;
    return sensors[sensor_idx].valid;
}

// 任务调度注册
// static void update_sensor_task()
// {
//     AP_MultiDistanceSensor::instance().update();
// }

// AP_SCHEDULER_TASK(update_sensor_task, 50, 100);