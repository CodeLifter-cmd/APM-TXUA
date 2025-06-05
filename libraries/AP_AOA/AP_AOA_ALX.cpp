// libraries/AP_AOA/AP_AOA_ALX.cpp
#include "AP_AOA_ALX.h"
#include <GCS_MAVLink/GCS.h> //地面站
// const AP_Param::GroupInfo AP_AOA_ALX::var_info[] = {
//     AP_GROUPINFO("UART_NUM", 1, AP_AOA_ALX, _uart_num, 3),
//     AP_GROUPEND};

AP_AOA_ALX::AP_AOA_ALX() : _payload_len(0),
                           _payload_cnt(0),
                           _xor_sum(0),
                           _parse_state(WAIT_HEADER1)
{
    memset(&_current, 0, sizeof(_current));
}

void AP_AOA_ALX::init(uint8_t sernum)
{
    _uart = hal.serial(sernum);
    _uart->begin(230400, 256, 256);
    _uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    _uart->set_stop_bits(1);
}

void AP_AOA_ALX::update()
{
    // gcs().send_text(MAV_SEVERITY_INFO, "观察传感器采集函数是否执行");
    uint8_t rec_num = _uart->available();
    // gcs().send_text(MAV_SEVERITY_INFO,"rec_num:%d", rec_num); // 发送监控参数指令
    while (rec_num > 0)
    {
        rec_num--;
        uint8_t byte = _uart->read();
        // gcs().send_text(MAV_SEVERITY_INFO, "byte:%02x", byte);
        // gcs().send_named_float("byte",byte);   //发送监控参数指令
        switch (_parse_state)
        {
        case WAIT_HEADER1:
            if (byte == 0xFF)
            {
                _parse_state = WAIT_HEADER2;
                _xor_sum = byte;
            }
            break;
        case WAIT_HEADER2:
            if (byte == 0xFF)
            {
                _parse_state = WAIT_HEADER3;
                _xor_sum ^= byte;
            }
            else
            {
                _reset_parser();
            }
            break;
        case WAIT_HEADER3:
            if (byte == 0xFF)
            {
                _parse_state = WAIT_HEADER4;
                _xor_sum ^= byte;
            }
            else
            {
                _reset_parser();
            }
            break;
        case WAIT_HEADER4:
            if (byte == 0xFF)
            {
                _parse_state = PARSE_LENGTH_H;
                _xor_sum ^= byte;
            }
            else
            {
                _reset_parser();
            }
            break;
        case PARSE_LENGTH_H:
            _payload_len = (byte << 8);
            _parse_state = PARSE_LENGTH_L;
            _xor_sum ^= byte;
            break;
        case PARSE_LENGTH_L:
            _payload_len |= byte;
            // gcs().send_text(MAV_SEVERITY_INFO, "PARSE_LENGTH_H:%d", _payload_len);
            if (_payload_len != AOA_MAX_PAYLOAD)  //数据长度不等于37则返回
            {
                _reset_parser();
                break;
            }
            // gcs().send_text(MAV_SEVERITY_INFO, "_payload_len:%d", _payload_len);
            _parse_state = PARSE_PAYLOAD;
            _payload_cnt = 0;
            _xor_sum ^= byte;
            break;
        case PARSE_PAYLOAD:
            _rx_buffer[_payload_cnt++] = byte;
            _xor_sum ^= byte;
            if (_payload_cnt >= _payload_len - 7)
            {
                _parse_state = CHECK_XOR;
            }
            break;
        case CHECK_XOR:
        {
            uint8_t calc_xor = _xor_sum;
            if (byte == calc_xor)
            {
                _process_packet();
            }
            else
            {
                gcs().send_text(MAV_SEVERITY_INFO, "AOA XOR Err:%02x vs %02x\n", byte, calc_xor);
            }
            _reset_parser();
            break;
        }
        default:
            _reset_parser();
            break;
        }
    }
}
void AP_AOA_ALX::_reset_parser()
{
    _payload_len = 0;
    _payload_cnt = 0;
    _xor_sum = 0;
    _parse_state = WAIT_HEADER1;
}
void AP_AOA_ALX::_process_packet()
{
    // 解析协议0x2001数据包
    if (_rx_buffer[2] == 0x20 && _rx_buffer[3] == 0x01)
    {
        // 距离解析 (cm转m)
        uint32_t dist_cm = (uint32_t)_rx_buffer[14] << 24 |
                           (uint32_t)_rx_buffer[15] << 16 |
                           (uint32_t)_rx_buffer[16] << 8 |
                           _rx_buffer[17];
        _current.distance_m = dist_cm * 0.01f;

        // 方位角解析 (int16_t)
        int16_t azimuth = (int16_t)(_rx_buffer[18] << 8 | _rx_buffer[19]);
        _current.azimuth_deg = azimuth;

        _current.timestamp_ms = AP_HAL::millis();
        _current.data_valid = true;
    }
}

bool AP_AOA_ALX::get_raw_data(float &dist_m, float &azimuth_deg)
{
    if (_current.data_valid && (AP_HAL::millis() - _current.timestamp_ms < 200))
    {
        dist_m = _current.distance_m;
        azimuth_deg = _current.azimuth_deg;
        _current.data_valid = false;
        return true;
    }
    return false;
}
