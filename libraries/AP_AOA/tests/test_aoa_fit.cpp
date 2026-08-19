#include <AP_gtest.h>

#include <AP_AOA/AP_AOA_ALX_FIT.h>

#include <array>
#include <cstdint>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

namespace {

constexpr size_t frame_length = 37;  // 0x2001 定位帧整包长度

using Frame = std::array<uint8_t, frame_length>;

// FIT 协议 XOR 校验：前 N-1 字节异或 == 第 N 字节
uint8_t fit_xor(const Frame &frame)
{
    uint8_t x = 0;
    for (size_t i = 0; i + 1 < frame.size(); i++) {
        x ^= frame[i];
    }
    return x;
}

// 依据数据手册 6.2 节构造 0x2001 定位帧（大端）
// 字段偏移：header[0..3] len[4..5] seq[6..7] cmd[8..9] ver[10..11]
//           anchor[12..15] tag[16..19] dist[20..23] az[24..25]
//           elev[26..27] status[28..29] batch[30..31] reserve[32..35] xor[36]
Frame make_fit_frame(uint16_t azimuth, uint32_t distance_cm = 125)
{
    Frame frame{};
    frame[0] = 0xFF;
    frame[1] = 0xFF;
    frame[2] = 0xFF;
    frame[3] = 0xFF;
    // PacketLength = 37（大端）
    frame[4] = 0x00;
    frame[5] = 0x25;
    // SequenceID = 0x2246（大端）
    frame[6] = 0x22;
    frame[7] = 0x46;
    // RequestCommand = 0x2001（大端）
    frame[8] = 0x20;
    frame[9] = 0x01;
    // VersionID = 0x0102
    frame[10] = 0x01;
    frame[11] = 0x02;
    // AnchorID = 100（BE32）
    frame[12] = 0x00;
    frame[13] = 0x00;
    frame[14] = 0x00;
    frame[15] = 0x64;
    // TagID = 100（BE32）
    frame[16] = 0x00;
    frame[17] = 0x00;
    frame[18] = 0x00;
    frame[19] = 0x64;
    // Distance（BE32，单位 cm）
    frame[20] = uint8_t((distance_cm >> 24) & 0xFFU);
    frame[21] = uint8_t((distance_cm >> 16) & 0xFFU);
    frame[22] = uint8_t((distance_cm >> 8) & 0xFFU);
    frame[23] = uint8_t(distance_cm & 0xFFU);
    // Azimuth（BE16，取值 0-359）
    frame[24] = uint8_t((azimuth >> 8) & 0xFFU);
    frame[25] = uint8_t(azimuth & 0xFFU);
    // Elevation（预留）
    frame[26] = 0x00;
    frame[27] = 0x00;
    // TagStatus
    frame[28] = 0x12;
    frame[29] = 0x34;
    // BatchSn = 0x106B
    frame[30] = 0x10;
    frame[31] = 0x6B;
    // Reserve
    frame[32] = 0xAC;
    frame[33] = 0x00;
    frame[34] = 0x00;
    frame[35] = 0x00;
    // XorByte
    frame[36] = fit_xor(frame);
    return frame;
}

} // namespace

// 数据手册 6.2 示例：距离 125cm 角度 328°
// FF FF FF FF 00 25 22 46 20 01 01 02 00 00 00 64 00 00 00 64
// 00 00 00 7D 01 48 00 00 12 34 10 6B AC 00 00 00 A6
TEST(AP_AOA_ALX_FIT, DecodesDatasheetExampleFrame)
{
    const Frame frame = make_fit_frame(328, 125);
    AP_AOA_ALX_FIT::DecodedFrame decoded{};

    ASSERT_TRUE(AP_AOA_ALX_FIT::decode_frame(frame.data(), frame.size(), decoded));
    EXPECT_EQ(125U, decoded.distance_cm);
    EXPECT_EQ(-32, decoded.azimuth_deg);   // 328 -> -32
    EXPECT_EQ(100U, decoded.tag_id);
    EXPECT_EQ(0x2246U, decoded.seq_id);
    EXPECT_EQ(0x106BU, decoded.batch_sn);
    EXPECT_EQ(100, decoded.confidence);
    EXPECT_EQ(0, decoded.rssi);
}

// 角度 0-359 -> ±180 转换
TEST(AP_AOA_ALX_FIT, AzimuthConversion0To359)
{
    struct TestCase {
        uint16_t in;
        int16_t out;
    };
    const TestCase cases[] = {
        {0, 0},
        {57, 57},
        {180, 180},
        {181, -179},
        {328, -32},
        {359, -1},
    };

    for (const auto &test_case : cases) {
        SCOPED_TRACE(test_case.in);
        const Frame frame = make_fit_frame(test_case.in);
        AP_AOA_ALX_FIT::DecodedFrame decoded{};
        ASSERT_TRUE(AP_AOA_ALX_FIT::decode_frame(frame.data(), frame.size(), decoded));
        EXPECT_EQ(test_case.out, decoded.azimuth_deg);
    }
}

// XOR 校验错误必须被拒绝
TEST(AP_AOA_ALX_FIT, RejectsBadXor)
{
    Frame frame = make_fit_frame(328);
    frame[36] ^= 0xFF;
    AP_AOA_ALX_FIT::DecodedFrame decoded{};

    EXPECT_FALSE(AP_AOA_ALX_FIT::decode_frame(frame.data(), frame.size(), decoded));
}

// 帧头错误必须被拒绝
TEST(AP_AOA_ALX_FIT, RejectsWrongHeader)
{
    Frame frame = make_fit_frame(328);
    frame[0] = 0xFE;
    frame[36] = fit_xor(frame);
    AP_AOA_ALX_FIT::DecodedFrame decoded{};

    EXPECT_FALSE(AP_AOA_ALX_FIT::decode_frame(frame.data(), frame.size(), decoded));
}

// 命令码非 0x2001（如 0x2002 心跳）必须被 decode_frame 拒绝
TEST(AP_AOA_ALX_FIT, RejectsHeartbeatCommand)
{
    Frame frame = make_fit_frame(0);
    frame[8] = 0x20;
    frame[9] = 0x02;
    frame[36] = fit_xor(frame);
    AP_AOA_ALX_FIT::DecodedFrame decoded{};

    EXPECT_FALSE(AP_AOA_ALX_FIT::decode_frame(frame.data(), frame.size(), decoded));
}

// 帧长不足 37 必须被拒绝（心跳 0x2002 仅 16 字节）
TEST(AP_AOA_ALX_FIT, RejectsShortFrame)
{
    std::array<uint8_t, 16> short_frame{};
    short_frame[0] = 0xFF;
    short_frame[1] = 0xFF;
    short_frame[2] = 0xFF;
    short_frame[3] = 0xFF;
    short_frame[4] = 0x00;
    short_frame[5] = 0x10;
    short_frame[8] = 0x20;
    short_frame[9] = 0x02;
    AP_AOA_ALX_FIT::DecodedFrame decoded{};

    EXPECT_FALSE(AP_AOA_ALX_FIT::decode_frame(short_frame.data(), short_frame.size(), decoded));
}

// 角度超出 0-359（如 360）必须被拒绝
TEST(AP_AOA_ALX_FIT, RejectsAzimuthOutOfRange)
{
    Frame frame = make_fit_frame(0);
    frame[24] = 0x01;
    frame[25] = 0x68;   // 0x0168 = 360
    frame[36] = fit_xor(frame);
    AP_AOA_ALX_FIT::DecodedFrame decoded{};

    EXPECT_FALSE(AP_AOA_ALX_FIT::decode_frame(frame.data(), frame.size(), decoded));
}

TEST(AP_AOA_ALX_FIT, ReportsAcceptedFrameWithMatchingRawSampleEvent)
{
    AP_AOA_ALX_FIT sensor;
    const AP_AOA_ALX_FIT::DecodedFrame frame{125, -32, 100, 0, 100, 0x2246, 1};

    ASSERT_TRUE(sensor.accept_decoded_frame(frame, 250U));

    AP_AOA_ALX_FIT::DiagnosticEvent event{};
    ASSERT_TRUE(sensor.get_diagnostic_event(event));
    EXPECT_EQ(1U, event.event_sequence);
    EXPECT_EQ(250U, event.timestamp_ms);
    EXPECT_EQ(AP_AOA_ALX_FIT::GateResult::ACCEPTED, event.result);
    EXPECT_EQ(125U, event.frame.distance_cm);
    EXPECT_EQ(-32, event.frame.azimuth_deg);
    EXPECT_EQ(100U, event.frame.tag_id);
    EXPECT_EQ(0x2246U, event.frame.seq_id);
    EXPECT_EQ(1U, event.frame.batch_sn);
    EXPECT_FALSE(sensor.get_diagnostic_event(event));

    float distance_m = 0.0f;
    float angle_deg = 0.0f;
    uint32_t timestamp_ms = 0;
    uint32_t event_sequence = 0;
    ASSERT_TRUE(sensor.get_raw_data(distance_m, angle_deg, &timestamp_ms, &event_sequence));
    EXPECT_FLOAT_EQ(1.25f, distance_m);
    EXPECT_FLOAT_EQ(-32.0f, angle_deg);
    EXPECT_EQ(250U, timestamp_ms);
    EXPECT_EQ(1U, event_sequence);
}

TEST(AP_AOA_ALX_FIT, ReportsGateRejectReasonAndPreservesRawFields)
{
    AP_AOA_ALX_FIT sensor;
    const AP_AOA_ALX_FIT::DecodedFrame invalid_distance{0, 45, 100, 0, 77, 9, 3};

    ASSERT_FALSE(sensor.accept_decoded_frame(invalid_distance, 300U));

    AP_AOA_ALX_FIT::DiagnosticEvent event{};
    ASSERT_TRUE(sensor.get_diagnostic_event(event));
    EXPECT_EQ(AP_AOA_ALX_FIT::GateResult::DISTANCE, event.result);
    EXPECT_EQ(0U, event.frame.distance_cm);
    EXPECT_EQ(45, event.frame.azimuth_deg);
    EXPECT_EQ(77U, event.frame.tag_id);
    EXPECT_EQ(9U, event.frame.seq_id);
    EXPECT_EQ(3U, event.frame.batch_sn);
    EXPECT_EQ(1U, sensor.gating_reject_count());
}

TEST(AP_AOA_ALX_FIT, ReportsSequenceAndDistanceStepRejectReasonsInOrder)
{
    AP_AOA_ALX_FIT sensor;
    ASSERT_TRUE(sensor.accept_decoded_frame({100, 0, 100, 0, 1, 1, 1}, 100U));
    ASSERT_FALSE(sensor.accept_decoded_frame({100, 1, 100, 0, 1, 2, 9}, 125U));
    ASSERT_FALSE(sensor.accept_decoded_frame({400, 2, 100, 0, 1, 3, 10}, 150U));

    AP_AOA_ALX_FIT::DiagnosticEvent event{};
    ASSERT_TRUE(sensor.get_diagnostic_event(event));
    EXPECT_EQ(AP_AOA_ALX_FIT::GateResult::ACCEPTED, event.result);
    ASSERT_TRUE(sensor.get_diagnostic_event(event));
    EXPECT_EQ(AP_AOA_ALX_FIT::GateResult::SEQUENCE, event.result);
    EXPECT_EQ(9U, event.frame.batch_sn);
    ASSERT_TRUE(sensor.get_diagnostic_event(event));
    EXPECT_EQ(AP_AOA_ALX_FIT::GateResult::DISTANCE_STEP, event.result);
    EXPECT_EQ(400U, event.frame.distance_cm);
}

TEST(AP_AOA_ALX_FIT, CountsOldestDiagnosticEventsDroppedOnQueueOverrun)
{
    AP_AOA_ALX_FIT sensor;
    for (uint16_t i = 1; i <= 32; i++) {
        ASSERT_TRUE(sensor.accept_decoded_frame({100, 0, 100, 0, 1, i, i}, i * 25U));
    }

    EXPECT_GT(sensor.diagnostic_drop_count(), 0U);
    AP_AOA_ALX_FIT::DiagnosticEvent event{};
    uint32_t previous_sequence = 0;
    while (sensor.get_diagnostic_event(event)) {
        EXPECT_GT(event.event_sequence, previous_sequence);
        previous_sequence = event.event_sequence;
    }
    EXPECT_EQ(32U, previous_sequence);
}

TEST(AP_AOA_ALX_FIT, ResetSessionDiscardsStaleSampleAndDiagnosticState)
{
    AP_AOA_ALX_FIT sensor;
    ASSERT_TRUE(sensor.accept_decoded_frame({125, -32, 100, 0, 100, 7, 42}, 250U));

    sensor.reset_session();

    float distance_m = 0.0f;
    float angle_deg = 0.0f;
    uint32_t timestamp_ms = 0;
    uint32_t event_sequence = 0;
    EXPECT_FALSE(sensor.get_raw_data(distance_m, angle_deg, &timestamp_ms, &event_sequence));
    EXPECT_EQ(UINT32_MAX, sensor.valid_frame_age_ms(1000U));
    EXPECT_EQ(0U, sensor.decode_error_count());
    EXPECT_EQ(0U, sensor.gating_reject_count());
    EXPECT_EQ(0U, sensor.gating_accept_count());
    EXPECT_EQ(0U, sensor.heartbeat_count());
    EXPECT_EQ(0U, sensor.diagnostic_drop_count());

    AP_AOA_ALX_FIT::DiagnosticEvent event{};
    EXPECT_FALSE(sensor.get_diagnostic_event(event));

    // A new session must accept a restarted BatchSn and expose only the fresh frame.
    ASSERT_TRUE(sensor.accept_decoded_frame({200, 5, 100, 0, 100, 1, 1}, 1000U));
    ASSERT_TRUE(sensor.get_raw_data(distance_m, angle_deg, &timestamp_ms, &event_sequence));
    EXPECT_FLOAT_EQ(2.0f, distance_m);
    EXPECT_FLOAT_EQ(5.0f, angle_deg);
    EXPECT_EQ(1000U, timestamp_ms);
    EXPECT_EQ(1U, event_sequence);
}

AP_GTEST_MAIN()
