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
    // _uart->begin(230400, 256, 256);
    _uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    _uart->set_stop_bits(1);
}

void AP_AOA_ALX::update()
{
    // gcs().send_text(MAV_SEVERITY_INFO, "观察传感器采集函数是否执行");
    uint8_t rec_num = _uart->available();

    gcs().send_text(MAV_SEVERITY_INFO,"rec_num:%d", rec_num); // 发送监控参数指令
    while (rec_num > 0)
    {
        rec_num--;
        uint8_t byte = _uart->read();
        // gcs().send_text(MAV_SEVERITY_INFO, "byte:%02x", byte);
        // gcs().send_named_float("byte",byte);   //发送监控参数指令
        switch (_parse_state)
        {
        case WAIT_HEADER1:
            if (byte == 0x59)  //帧头0✖59 B1
            {
                _xor_sum = 0;
                _rx_buffer[0] = byte;
                _xor_sum += byte;
                _parse_state = WAIT_HEADER2;
            }
            break;
        case WAIT_HEADER2:
            if (byte == 0x4D)  //帧头0✖40 B2
            {
                _rx_buffer[1] = byte;
                _xor_sum += byte;
                _parse_state = WAIT_HEADER3;
            }
            else
            {
                _reset_parser();
            }
            break;
        case WAIT_HEADER3:
            if (byte == 0x17)  //PDOA跟随模式0X17 B3
            {
                _rx_buffer[2] = byte;
                _xor_sum += byte;
                _parse_state = SEQ_B4;
            }
            else
            {
                _reset_parser();
            }
            break;
        case SEQ_B4: // 帧序列号
            _rx_buffer[3] = byte;
            _xor_sum += byte;
            _parse_state = SEQ_B5;
            break;
        case SEQ_B5: // 帧序列号
            _rx_buffer[4] = byte;
            _xor_sum += byte;
            _parse_state = PARSE_LENGTH_L;
            break;
        case PARSE_LENGTH_L:
            _rx_buffer[5] = byte;
            _payload_len = byte;
            _xor_sum += byte;
            _parse_state = PARSE_LENGTH_H;
            break;
        case PARSE_LENGTH_H:
            _rx_buffer[6] = byte;
            _payload_len |= (byte << 8);
            _xor_sum += byte;
            // gcs().send_text(MAV_SEVERITY_INFO, "PARSE_LENGTH_H:%d", _payload_len);
            if (_payload_len != AOA_MAX_PAYLOAD)  //数据长度不等于37则返回
            {
                _reset_parser();
                break;
            }
            // gcs().send_text(MAV_SEVERITY_INFO, "_payload_len:%d", _payload_len);
            _parse_state = PARSE_PAYLOAD;
            _payload_cnt = 7; //前7个字节已存储

            break;
        case PARSE_PAYLOAD:
            _rx_buffer[_payload_cnt++] = byte;
            _xor_sum += byte;
            if (_payload_cnt >= _payload_len + 7)
            {
                _parse_state = CHECK_SUM;
            }
            break;
        case CHECK_SUM:
        {
            uint8_t calc_xor = _xor_sum;
            if (byte == calc_xor)
            {
                _process_packet();
            }
            else
            {
                gcs().send_text(MAV_SEVERITY_INFO, "AOA XOR Err:%02x vs %02x\n", byte, calc_xor);
                // gcs().send_text(MAV_SEVERITY_INFO, "_rx_buffer:%02x,%02x,%02x,%02x,%02x,%02x\n", _rx_buffer[125], _rx_buffer[124], _rx_buffer[123], _rx_buffer[122], _rx_buffer[121], _rx_buffer[120]);
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
    // 解析协议0x17数据包
    if (_rx_buffer[2] == 0x17)
    {
        // 距离解析 (cm转m)
        uint32_t dist_cm = (uint32_t)_rx_buffer[15] << 24 |
                           (uint32_t)_rx_buffer[14] << 16 |
                           (uint32_t)_rx_buffer[13] << 8 |
                           _rx_buffer[12];
        _current.distance_m = dist_cm * 0.01f;

        // 方位角解析 (int16_t)
        int16_t azimuth = (int16_t)(_rx_buffer[18] << 8 | _rx_buffer[17]);
        _current.azimuth_deg = azimuth;

        _current.timestamp_ms = AP_HAL::millis();
        _current.data_confirmed = _rx_buffer[19];
        _current.data_RSSI = _rx_buffer[21];
        gcs().send_named_float("data_confirmed", _current.data_confirmed);
        gcs().send_named_float("data_RSSI", _current.data_RSSI);
    }
}

bool AP_AOA_ALX::get_raw_data(float &dist_m, float &azimuth_deg)
{
    if ((_current.data_confirmed > 50) && (AP_HAL::millis() - _current.timestamp_ms < 1000))
    {
        dist_m = _current.distance_m;
        azimuth_deg = _current.azimuth_deg;
        _current.data_confirmed = 0;
        return true;
    }
    return false;
}
