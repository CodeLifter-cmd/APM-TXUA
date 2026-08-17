// libraries/AP_AOA/AP_AOA_ALX_FIT.cpp
// ALX-AOA-FIT 全向360°基站&信标 驱动实现（v1.1 方案）
#include "AP_AOA_ALX_FIT.h"
#include <GCS_MAVLink/GCS.h> //地面站

// ── 大端解码内联 ──────────────────────────────────────────
static inline uint16_t be16(const uint8_t *p)
{
    return uint16_t(p[0]) << 8 | p[1];
}
static inline uint32_t be32(const uint8_t *p)
{
    return uint32_t(p[0]) << 24 | uint32_t(p[1]) << 16 |
           uint32_t(p[2]) << 8 | p[3];
}

AP_AOA_ALX_FIT::AP_AOA_ALX_FIT() : _uart(nullptr),
                                   _packet_len(0),
                                   _payload_cnt(0),
                                   _payload_remaining(0),
                                   _parse_state(WAIT_FF1),
                                   _sample_sequence(0),
                                   _consumed_sequence(0),
                                   _last_error_report_ms(0),
                                   _decode_error_count(0),
                                   _gating_reject_count(0),
                                   _gating_accept_count(0),
                                   _heartbeat_count(0),
                                   _last_heartbeat_ms(0),
                                   _last_seq_id(0),
                                   _have_seen_sn(false),
                                   _last_seen_batch_sn(0),
                                   _last_seen_dist_cm(0)
{
    memset(&_current, 0, sizeof(_current));
}

void AP_AOA_ALX_FIT::init(uint8_t sernum)
{
    _uart = hal.serial(sernum);
    _uart->begin(115200, 256, 256);   // 波特率 115200（驱动硬编码，MP 参数不控制）
    _uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    _uart->set_stop_bits(1);
}

void AP_AOA_ALX_FIT::_reset_parser()
{
    _packet_len = 0;
    _payload_cnt = 0;
    _payload_remaining = 0;
    _parse_state = WAIT_FF1;
}

void AP_AOA_ALX_FIT::update()
{
    uint32_t bytes_available = _uart->available();

    while (bytes_available > 0)
    {
        bytes_available--;
        const uint8_t byte = _uart->read();

        switch (_parse_state)
        {
        case WAIT_FF1:
            if (byte == 0xFF)   // 帧头第 1 个 0xFF
            {
                _rx_buffer[0] = byte;
                _parse_state = WAIT_FF2;
            }
            break;
        case WAIT_FF2:
            if (byte == 0xFF)
            {
                _rx_buffer[1] = byte;
                _parse_state = WAIT_FF3;
            }
            else
            {
                _reset_parser();
                // 避免漏掉"FF FF FF"边界：当前字节也可能是新帧头第 1 字节
                if (byte == 0xFF)
                {
                    _rx_buffer[0] = byte;
                    _parse_state = WAIT_FF2;
                }
            }
            break;
        case WAIT_FF3:
            if (byte == 0xFF)
            {
                _rx_buffer[2] = byte;
                _parse_state = WAIT_FF4;
            }
            else
            {
                _reset_parser();
            }
            break;
        case WAIT_FF4:
            if (byte == 0xFF)
            {
                _rx_buffer[3] = byte;
                _parse_state = PARSE_LENGTH_L;
            }
            else
            {
                _reset_parser();
            }
            break;
        case PARSE_LENGTH_L:  // PacketLength 高字节（大端）
            _rx_buffer[4] = byte;
            _parse_state = PARSE_LENGTH_H;
            break;
        case PARSE_LENGTH_H:  // PacketLength 低字节
        {
            _rx_buffer[5] = byte;
            _packet_len = be16(&_rx_buffer[4]);
            // L2: 长度合理性（0x2001=37 / 0x2002=16，均 ≥14 且 ≤ FIT_MAX）
            if (_packet_len < 14 || _packet_len > AOA_FIT_MAX_FRAME)
            {
                _reset_parser();
                break;
            }
            _payload_cnt = 6;                  // 已收 6 字节（4 头 + 2 长度）
            _payload_remaining = _packet_len - 6;
            if (_payload_remaining == 0)
            {
                // 理论不可达（packet_len ≥ 14），防御性处理
                _reset_parser();
                break;
            }
            _parse_state = PARSE_PAYLOAD;
            break;
        }
        case PARSE_PAYLOAD:
        {
            _rx_buffer[_payload_cnt++] = byte;
            if (--_payload_remaining == 0)
            {
                // ★ v1.1 关键修正：收满 packet_len 字节即完成一帧，
                //   _rx_buffer[_packet_len-1] 就是 XOR 校验字节，无需再读。
                // 0x2002 心跳帧（16 字节，无 XOR）：识别后丢弃，不触发定位更新
                if (_packet_len >= 10 && be16(&_rx_buffer[8]) == 0x2002)
                {
                    _heartbeat_count++;
                    _last_heartbeat_ms = AP_HAL::millis();
                    _reset_parser();
                    break;
                }
                DecodedFrame decoded{};
                const bool decoded_ok = decode_frame(_rx_buffer, _packet_len, decoded);
                if (decoded_ok)
                {
                    _process_packet(decoded);
                }
                else
                {
                    _decode_error_count++;
                    const uint32_t now_ms = AP_HAL::millis();
                    if (now_ms - _last_error_report_ms >= 1000)
                    {
                        _last_error_report_ms = now_ms;
                        gcs().send_text(MAV_SEVERITY_WARNING,
                                        "AOA FIT decode errors: %lu",
                                        (unsigned long)_decode_error_count);
                        _decode_error_count = 0;
                    }
                }
                _reset_parser();
            }
            break;
        }
        default:
            _reset_parser();
            break;
        }
    }
}

bool AP_AOA_ALX_FIT::DecodedFrame::is_acceptable() const
{
    // confidence 在 FIT 驱动中恒为 100（门控通过标记），质量由 L1-L9 承担
    return AP_AOA_ALX_FIT::confidence_is_acceptable(confidence);
}

bool AP_AOA_ALX_FIT::confidence_is_acceptable(uint8_t confidence)
{
    return confidence > 50;
}

bool AP_AOA_ALX_FIT::decode_frame(const uint8_t *frame, uint16_t frame_len, DecodedFrame &decoded)
{
    if (frame == nullptr || frame_len < 37) {
        return false;
    }
    // L1: 帧头 4×0xFF
    if (frame[0] != 0xFF || frame[1] != 0xFF ||
        frame[2] != 0xFF || frame[3] != 0xFF) {
        return false;
    }
    // L2: PacketLength 语义 = 整包长度（帧头 4 + 后续 33），0x2001 固定 37
    const uint16_t packet_len = be16(&frame[4]);
    if (packet_len < 37 || packet_len > AOA_FIT_MAX_FRAME ||
        packet_len > frame_len) {
        return false;
    }
    // L4: 命令码 0x2001（0x2002 心跳在 _process_packet 前拦截）
    if (be16(&frame[8]) != 0x2001) {
        return false;
    }
    // L3: XOR 校验 —— 前 packet_len-1 字节异或 == 最后一字节
    {
        uint8_t x = 0;
        for (uint16_t i = 0; i + 1 < packet_len; i++) {
            x ^= frame[i];
        }
        if (x != frame[packet_len - 1]) {
            return false;
        }
    }
    // 大端字段解码
    decoded.distance_cm = be32(&frame[20]);
    decoded.tag_id     = be32(&frame[16]);
    decoded.seq_id     = be16(&frame[6]);
    decoded.batch_sn   = be16(&frame[30]);
    // L6: 角度值域 0-359（uint16 BE），驱动层转 ±180
    const uint16_t raw_azimuth = be16(&frame[24]);
    if (raw_azimuth > 359) {
        return false;
    }
    decoded.azimuth_deg = raw_azimuth > 180 ?
                          int16_t(raw_azimuth - 360) : int16_t(raw_azimuth);
    decoded.confidence = 100;   // L1-L4/L6 通过标记
    decoded.rssi = 0;           // 新模块无 RSSI
    return true;
}

void AP_AOA_ALX_FIT::_process_packet(const DecodedFrame &decoded)
{
    // 0x2001 定位帧：走接受门控（L5/L7/L8/L9）；
    // 0x2002 心跳已在 update() 的收帧完成处提前拦截，不进入本函数。
    accept_decoded_frame(decoded, AP_HAL::millis());
}

bool AP_AOA_ALX_FIT::accept_decoded_frame(const DecodedFrame &decoded, uint32_t timestamp_ms)
{
    // L5: 距离合理性（0 < d ≤ MAXD）
    if (decoded.distance_cm == 0 || decoded.distance_cm > AOA_FIT_MAXD_CM) {
        _gating_reject_count++;
        return false;
    }
    // L7: TagID 匹配（0=不校验）
    if (AOA_FIT_TAGID != 0 && decoded.tag_id != AOA_FIT_TAGID) {
        _gating_reject_count++;
        return false;
    }
    // L8: BatchSn 连续性 —— 与"最近一次见到的帧"比（缺失允许 SNGAP 内）
    if (_have_seen_sn) {
        const int32_t diff = int32_t(decoded.batch_sn) - int32_t(_last_seen_batch_sn);
        if (diff < 1 || diff > AOA_FIT_SNGAP) {
            // 序列倒退或跳跃过大 → 拒收，但用本帧刷新基线，防连锁拒收
            _last_seen_batch_sn = decoded.batch_sn;
            _gating_reject_count++;
            return false;
        }
    }
    // L9: 距离变化率 —— 相对"最近一次见到的帧"
    if (_have_seen_sn) {
        const uint32_t d0 = _last_seen_dist_cm;
        const uint32_t d1 = decoded.distance_cm;
        const uint32_t delta = d1 > d0 ? d1 - d0 : d0 - d1;
        if (delta > AOA_FIT_DSTEP_CM) {
            _last_seen_batch_sn = decoded.batch_sn;
            _last_seen_dist_cm = decoded.distance_cm;
            _gating_reject_count++;
            return false;
        }
    }

    // ── 全部门控通过 → 接受 ──
    _current.distance_m = decoded.distance_cm * 0.01f;
    _current.azimuth_deg = decoded.azimuth_deg;
    _current.timestamp_ms = timestamp_ms;
    _current.data_confirmed = decoded.confidence;
    _current.data_RSSI = decoded.rssi;
    _sample_sequence++;
    _gating_accept_count++;
    // 更新 L8/L9 基线
    _have_seen_sn = true;
    _last_seen_batch_sn = decoded.batch_sn;
    _last_seen_dist_cm = decoded.distance_cm;
    _last_seq_id = decoded.seq_id;
    return true;
}

uint32_t AP_AOA_ALX_FIT::valid_frame_age_ms(uint32_t now_ms) const
{
    return _sample_sequence == 0 ? UINT32_MAX : now_ms - _current.timestamp_ms;
}

bool AP_AOA_ALX_FIT::get_raw_data(float &dist_m, float &azimuth_deg, uint32_t *timestamp_ms)
{
    if (_consumed_sequence != _sample_sequence)
    {
        dist_m = _current.distance_m;
        azimuth_deg = _current.azimuth_deg;
        if (timestamp_ms != nullptr)
        {
            *timestamp_ms = _current.timestamp_ms;
        }
        _consumed_sequence = _sample_sequence;
        return true;
    }
    return false;
}
