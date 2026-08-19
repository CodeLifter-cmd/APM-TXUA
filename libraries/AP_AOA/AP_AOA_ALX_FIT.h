// libraries/AP_AOA/AP_AOA_ALX_FIT.h
// ALX-AOA-FIT 全向360°基站&信标 驱动（v1.1 方案）
// 协议：帧头 4×0xFF + PacketLength(大端,整包37) + ... + XOR校验(最后一字节)
// 关键差异（相对旧 AP_AOA_ALX）：
//   1) 字节序大端 BE16/BE32
//   2) 37 字节整包，XOR 是收满 packet_len 后的最后一字节，无独立 PARSE_XOR 状态
//   3) 角度 uint16 0-359（无符号，BE），驱动内转 ±180
//   4) 无置信度/RSSI → 多层门控 L1-L9 替代（解码层 L1/L2/L3/L4/L6 + 接受层 L5/L7/L8/L9）
//   5) 心跳帧 0x2002 每 2s，无 XOR，识别后丢弃
#pragma once
#include <AP_HAL/AP_HAL.h>

#define AOA_FIT_MAX_FRAME 64    // 最大帧缓冲（0x2001=37，余量兼容厂家扩展）

// 门控参数（v1 编译期常量；标定后可升级为 AP_Param）
#define AOA_FIT_MAXD_CM   2000  // L5: 最大可信距离 cm
#define AOA_FIT_TAGID     0     // L7: 期望标签 ID，0=不校验
#define AOA_FIT_SNGAP     3     // L8: BatchSn 相对上一帧允许最大间隔
#define AOA_FIT_DSTEP_CM  150   // L9: 相对上一帧距离变化上限 cm

class AP_AOA_ALX_FIT
{
public:
    struct DecodedFrame {
        uint32_t distance_cm;   // 距离 cm（BE32, frame[20..23]）
        int16_t azimuth_deg;    // 方位角 ±180（已从 0-359 转换）
        uint8_t confidence;     // 通过 L1-L6 校验 = 100（质量由门控链承担）
        uint8_t rssi;           // 新模块无此字段，恒 0
        uint32_t tag_id;        // TagID（BE32, frame[16..19]）
        uint16_t seq_id;        // SequenceID（BE16, frame[6..7]）
        uint16_t batch_sn;      // BatchSn（BE16, frame[30..31]）

        bool is_acceptable() const;
    };

    enum class GateResult : uint8_t {
        ACCEPTED = 0,
        DISTANCE = 1,
        TAG_ID = 2,
        SEQUENCE = 3,
        DISTANCE_STEP = 4,
        DECODE_ERROR = 5,
        HEARTBEAT = 6,
    };

    struct DiagnosticEvent {
        DecodedFrame frame{};
        uint32_t timestamp_ms = 0;
        uint32_t event_sequence = 0;
        uint32_t decode_errors = 0;
        uint32_t gating_rejects = 0;
        uint32_t gating_accepts = 0;
        uint32_t heartbeats = 0;
        uint32_t queue_drops = 0;
        GateResult result = GateResult::DECODE_ERROR;
    };

    AP_AOA_ALX_FIT();
    void init(uint8_t sernum);
    void reset_session();
    void update();
    bool get_raw_data(float &dist_m, float &azimuth_deg, uint32_t *timestamp_ms = nullptr,
                      uint32_t *event_sequence = nullptr);
    bool get_diagnostic_event(DiagnosticEvent &event);
    bool accept_decoded_frame(const DecodedFrame &decoded, uint32_t timestamp_ms);
    uint32_t valid_frame_age_ms(uint32_t now_ms) const;
    uint8_t last_confidence() const { return _current.data_confirmed; }
    static bool decode_frame(const uint8_t *frame, uint16_t frame_len, DecodedFrame &decoded);
    static bool confidence_is_acceptable(uint8_t confidence);

    // 调试计数（GCS 消息栏可用）
    uint32_t decode_error_count() const { return _decode_error_count; }
    uint32_t gating_reject_count() const { return _gating_reject_count; }
    uint32_t gating_accept_count() const { return _gating_accept_count; }
    uint32_t heartbeat_count() const { return _heartbeat_count; }
    uint16_t last_sequence_id() const { return _last_seq_id; }
    uint32_t diagnostic_drop_count() const { return _diagnostic_drop_count; }

private:
    enum ParseState
    {
        WAIT_FF1,        // 帧头第 1 个 0xFF
        WAIT_FF2,        // 帧头第 2 个 0xFF
        WAIT_FF3,        // 帧头第 3 个 0xFF
        WAIT_FF4,        // 帧头第 4 个 0xFF
        PARSE_LENGTH_L,  // PacketLength 高字节（大端！）
        PARSE_LENGTH_H,  // PacketLength 低字节
        PARSE_PAYLOAD    // 剩余字节，收满 packet_len 即完成（XOR 在最后一字节）
    };
    const AP_HAL::HAL &hal = AP_HAL::get_HAL();
    AP_HAL::UARTDriver *_uart;
    uint8_t _rx_buffer[AOA_FIT_MAX_FRAME];
    uint16_t _packet_len;          // BE 解析出的整包长度
    uint16_t _payload_cnt;         // 已收字节数
    uint16_t _payload_remaining;   // 剩余待收字节数
    ParseState _parse_state;

    struct
    {
        uint32_t timestamp_ms;
        uint32_t event_sequence;
        float distance_m;       // 距离，单位米
        float azimuth_deg;      // 方位角，单位度（±180）
        uint8_t data_confirmed; // 数据可信度（=100，门控通过标记）
        uint8_t data_RSSI;      // 无此字段，恒 0
    } _current;
    uint32_t _sample_sequence;
    uint32_t _consumed_sequence;
    uint32_t _last_error_report_ms;
    uint32_t _decode_error_count;
    uint32_t _gating_reject_count;
    uint32_t _gating_accept_count;
    uint32_t _heartbeat_count;
    uint32_t _last_heartbeat_ms;
    uint16_t _last_seq_id;      // 最近接受帧 SequenceID

    static constexpr uint8_t DIAGNOSTIC_QUEUE_SIZE = 16;
    DiagnosticEvent _diagnostic_queue[DIAGNOSTIC_QUEUE_SIZE]{};
    uint32_t _diagnostic_event_sequence;
    uint32_t _diagnostic_drop_count;
    uint8_t _diagnostic_read_index;
    uint8_t _diagnostic_write_index;
    uint8_t _diagnostic_count;
    // L8/L9 门控基线（相对"上一帧"比较，每次调用更新，防止连锁拒收死锁）
    bool     _have_seen_sn;
    uint16_t _last_seen_batch_sn;
    uint32_t _last_seen_dist_cm;

    void _process_packet(const DecodedFrame &decoded);
    void _reset_parser();
    uint32_t _record_diagnostic(const DecodedFrame *frame, GateResult result,
                                uint32_t timestamp_ms);
};
