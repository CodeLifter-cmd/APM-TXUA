// libraries/AP_AOA/AP_AOA_ALX.h
#pragma once
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#define AOA_MAX_PAYLOAD 118
class AP_AOA_ALX
{
public:
    struct DecodedFrame {
        uint32_t distance_cm;
        int16_t azimuth_deg;
        uint8_t confidence;
        uint8_t rssi;

        bool is_acceptable() const;
    };

    AP_AOA_ALX();
    void init(uint8_t sernum);
    void update();
    bool get_raw_data(float &dist_m, float &azimuth_deg, uint32_t *timestamp_ms = nullptr);
    bool accept_decoded_frame(const DecodedFrame &decoded, uint32_t timestamp_ms);
    uint32_t valid_frame_age_ms(uint32_t now_ms) const;
    uint8_t last_confidence() const { return _current.data_confirmed; }
    static bool decode_frame(const uint8_t *frame, uint16_t frame_len, DecodedFrame &decoded);
    static bool confidence_is_acceptable(uint8_t confidence);

    // // 参数表
    // static const struct AP_Param::GroupInfo var_info[];

private:
    enum ParseState
    {
        WAIT_HEADER1,
        WAIT_HEADER2,
        WAIT_HEADER3,
        SEQ_B4,
        SEQ_B5,
        PARSE_LENGTH_L,
        PARSE_LENGTH_H,
        PARSE_PAYLOAD,
        CHECK_SUM
    };
    const AP_HAL::HAL &hal = AP_HAL::get_HAL();
    AP_HAL::UARTDriver *_uart;
    uint8_t _rx_buffer[AOA_MAX_PAYLOAD+10];
    uint16_t _payload_len;
    uint16_t _payload_cnt;
    uint8_t _xor_sum;
    ParseState _parse_state;

    struct
    {
        uint32_t timestamp_ms;
        float distance_m;    //距离，单位米
        float azimuth_deg;   //方位角，单位度
        uint8_t data_confirmed; //数据可信度
        uint8_t data_RSSI;      //信号强度

    } _current;
    uint32_t _sample_sequence;
    uint32_t _consumed_sequence;
    uint32_t _last_error_report_ms;
    uint32_t _decode_error_count;

    void _process_packet(const DecodedFrame &decoded);
    void _reset_parser();
};
