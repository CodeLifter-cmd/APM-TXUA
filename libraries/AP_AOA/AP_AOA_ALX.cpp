// libraries/AP_AOA/AP_AOA_ALX.cpp
#include "AP_AOA_ALX.h"
#include <GCS_MAVLink/GCS.h> //地面站
// const AP_Param::GroupInfo AP_AOA_ALX::var_info[] = {
//     AP_GROUPINFO("UART_NUM", 1, AP_AOA_ALX, _uart_num, 3),
//     AP_GROUPEND};

AP_AOA_ALX::AP_AOA_ALX() : _uart(nullptr),
                           _payload_len(0),
                           _payload_cnt(0),
                           _xor_sum(0),
                           _parse_state(WAIT_HEADER1),
                           _sample_sequence(0),
                           _consumed_sequence(0),
                           _last_error_report_ms(0),
                           _decode_error_count(0)
{
    memset(&_current, 0, sizeof(_current));
}

void AP_AOA_ALX::init(uint8_t sernum)
{
    _uart = hal.serial(sernum);
    _uart->begin(115200, 256, 256);   // 波特率 115200
    _uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    _uart->set_stop_bits(1);
}

void AP_AOA_ALX::update()
{
    // gcs().send_text(MAV_SEVERITY_INFO, "观察传感器采集函数是否执行");
    uint32_t bytes_available = _uart->available();

    while (bytes_available > 0)
    {
        bytes_available--;
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
            _rx_buffer[_payload_len + 7] = byte;
            DecodedFrame decoded{};
            if (decode_frame(_rx_buffer, _payload_len + 8, decoded))
            {
                _process_packet(decoded);
            }
            else
            {
                _decode_error_count++;
                const uint32_t now_ms = AP_HAL::millis();
                if (now_ms - _last_error_report_ms >= 1000) {
                    _last_error_report_ms = now_ms;
                    gcs().send_text(MAV_SEVERITY_WARNING, "AOA decode errors: %lu", (unsigned long)_decode_error_count);
                    _decode_error_count = 0;
                }
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

bool AP_AOA_ALX::DecodedFrame::is_acceptable() const
{
    return AP_AOA_ALX::confidence_is_acceptable(confidence);
}

bool AP_AOA_ALX::confidence_is_acceptable(uint8_t confidence)
{
    return confidence > 50;
}

bool AP_AOA_ALX::decode_frame(const uint8_t *frame, uint16_t frame_len, DecodedFrame &decoded)
{
    constexpr uint16_t frame_length = AOA_MAX_PAYLOAD + 8;
    if (frame == nullptr || frame_len != frame_length ||
        frame[0] != 0x59 || frame[1] != 0x4D || frame[2] != 0x17) {
        return false;
    }

    const uint16_t payload_len = uint16_t(frame[5]) | (uint16_t(frame[6]) << 8);
    if (payload_len != AOA_MAX_PAYLOAD) {
        return false;
    }

    uint8_t checksum = 0;
    for (uint16_t i = 0; i < frame_length - 1; i++) {
        checksum += frame[i];
    }
    if (checksum != frame[frame_length - 1]) {
        return false;
    }

    decoded.distance_cm = uint32_t(frame[12]) |
                          (uint32_t(frame[13]) << 8) |
                          (uint32_t(frame[14]) << 16) |
                          (uint32_t(frame[15]) << 24);
    const uint16_t raw_azimuth = uint16_t(frame[17]) | (uint16_t(frame[18]) << 8);
    decoded.azimuth_deg = raw_azimuth < 0x8000U ?
                          int16_t(raw_azimuth) : int16_t(int32_t(raw_azimuth) - 0x10000L);
    decoded.confidence = frame[19];
    decoded.rssi = frame[21];
    return true;
}

void AP_AOA_ALX::_process_packet(const DecodedFrame &decoded)
{
    accept_decoded_frame(decoded, AP_HAL::millis());
}

bool AP_AOA_ALX::accept_decoded_frame(const DecodedFrame &decoded, uint32_t timestamp_ms)
{
    if (!decoded.is_acceptable() || decoded.distance_cm == 0) {
        return false;
    }
    _current.distance_m = decoded.distance_cm * 0.01f;
    _current.azimuth_deg = decoded.azimuth_deg;
    _current.timestamp_ms = timestamp_ms;
    _current.data_confirmed = decoded.confidence;
    _current.data_RSSI = decoded.rssi;
    _sample_sequence++;
    return true;
}

uint32_t AP_AOA_ALX::valid_frame_age_ms(uint32_t now_ms) const
{
    return _sample_sequence == 0 ? UINT32_MAX : now_ms - _current.timestamp_ms;
}

bool AP_AOA_ALX::get_raw_data(float &dist_m, float &azimuth_deg, uint32_t *timestamp_ms)
{
    if (_consumed_sequence != _sample_sequence) {
        dist_m = _current.distance_m;
        azimuth_deg = _current.azimuth_deg;
        if (timestamp_ms != nullptr) {
            *timestamp_ms = _current.timestamp_ms;
        }
        _consumed_sequence = _sample_sequence;
        return true;
    }
    return false;
}
