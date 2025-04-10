// libraries/AP_AOA/AP_AOA_ALX.h
#pragma once
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#define AOA_MAX_PAYLOAD 37
class AP_AOA_ALX
{
public:
    AP_AOA_ALX();
    void init(uint8_t sernum);
    void update();
    bool get_raw_data(float &dist_m, float &azimuth_deg);

    // // 参数表
    // static const struct AP_Param::GroupInfo var_info[];

private:
    enum ParseState
    {
        WAIT_HEADER1,
        WAIT_HEADER2,
        WAIT_HEADER3,
        WAIT_HEADER4,
        PARSE_LENGTH_H,
        PARSE_LENGTH_L,
        PARSE_PAYLOAD,
        CHECK_XOR
    };
    const AP_HAL::HAL &hal = AP_HAL::get_HAL();
    AP_HAL::UARTDriver *_uart;
    uint8_t _rx_buffer[AOA_MAX_PAYLOAD];
    uint16_t _payload_len;
    uint16_t _payload_cnt;
    uint8_t _xor_sum;
    ParseState _parse_state;

    struct
    {
        uint32_t timestamp_ms;
        float distance_m;
        float azimuth_deg;
        bool data_valid;
    } _current;

    void _process_packet();
    void _reset_parser();
};
