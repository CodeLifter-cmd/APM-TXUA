#include <AP_gtest.h>

#include <AP_AOA/AP_AOA_ALX.h>

#include <array>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

namespace {

constexpr size_t frame_length = 126;
constexpr uint16_t payload_length = 118;

using Frame = std::array<uint8_t, frame_length>;

uint8_t frame_checksum(const Frame &frame)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < frame.size() - 1; i++) {
        checksum += frame[i];
    }
    return checksum;
}

Frame make_protocol_frame(int16_t azimuth, uint8_t confidence = 51,
                          uint8_t rssi = 213)
{
    Frame frame{};
    frame[0] = 0x59;
    frame[1] = 0x4D;
    frame[2] = 0x17;
    frame[3] = 0x34;
    frame[4] = 0x12;
    frame[5] = payload_length & 0xFF;
    frame[6] = payload_length >> 8;

    // Known protocol fields plus distinct neighbouring bytes catch offsets/swaps.
    frame[11] = 0xA1;
    frame[12] = 0x78;
    frame[13] = 0x56;
    frame[14] = 0x34;
    frame[15] = 0x12;
    frame[16] = 0xB2;
    frame[17] = static_cast<uint16_t>(azimuth) & 0xFF;
    frame[18] = static_cast<uint16_t>(azimuth) >> 8;
    frame[19] = confidence;
    frame[20] = 0xC3;
    frame[21] = rssi;
    frame[22] = 0xD4;
    frame.back() = frame_checksum(frame);
    return frame;
}

} // namespace

TEST(AP_AOA_ALX_V5, DecodesFullWidthSignedAzimuth)
{
    struct TestCase {
        int16_t encoded;
        int16_t expected;
    };
    const TestCase cases[] = {
        {57, 57},
        {241, 241},
        {256, 256},
        {265, 265},
        {-51, -51},
        {-87, -87},
    };

    for (const auto &test_case : cases) {
        SCOPED_TRACE(test_case.expected);
        const Frame frame = make_protocol_frame(test_case.encoded);
        AP_AOA_ALX::DecodedFrame decoded{};

        ASSERT_TRUE(AP_AOA_ALX::decode_frame(frame.data(), frame.size(), decoded));
        EXPECT_EQ(test_case.expected, decoded.azimuth_deg);
    }
}

TEST(AP_AOA_ALX_V5, DecodesFieldsAtDocumentedOffsets)
{
    const Frame frame = make_protocol_frame(265, 51, 213);
    AP_AOA_ALX::DecodedFrame decoded{};

    ASSERT_TRUE(AP_AOA_ALX::decode_frame(frame.data(), frame.size(), decoded));
    EXPECT_EQ(0x12345678U, decoded.distance_cm);
    EXPECT_EQ(265, decoded.azimuth_deg);
    EXPECT_EQ(51, decoded.confidence);
    EXPECT_EQ(213, decoded.rssi);
}

TEST(AP_AOA_ALX_V5, ConfidenceBoundaryRejects50AndAccepts51)
{
    AP_AOA_ALX::DecodedFrame decoded{};
    Frame frame = make_protocol_frame(57, 50);

    ASSERT_TRUE(AP_AOA_ALX::decode_frame(frame.data(), frame.size(), decoded));
    EXPECT_FALSE(decoded.is_acceptable());

    frame = make_protocol_frame(57, 51);
    ASSERT_TRUE(AP_AOA_ALX::decode_frame(frame.data(), frame.size(), decoded));
    EXPECT_TRUE(decoded.is_acceptable());
}

TEST(AP_AOA_ALX_V5, RejectsLowConfidenceBadAzimuthSample)
{
    const Frame frame = make_protocol_frame(-4005, 0);
    AP_AOA_ALX::DecodedFrame decoded{};

    ASSERT_TRUE(AP_AOA_ALX::decode_frame(frame.data(), frame.size(), decoded));
    EXPECT_EQ(-4005, decoded.azimuth_deg);
    EXPECT_FALSE(decoded.is_acceptable());
}

TEST(AP_AOA_ALX_V5, RejectsWrongHeader)
{
    Frame frame = make_protocol_frame(57);
    frame[0] = 0x58;
    frame.back() = frame_checksum(frame);
    AP_AOA_ALX::DecodedFrame decoded{};

    EXPECT_FALSE(AP_AOA_ALX::decode_frame(frame.data(), frame.size(), decoded));
}

TEST(AP_AOA_ALX_V5, RejectsWrongPayloadLength)
{
    Frame frame = make_protocol_frame(57);
    frame[5] = 117;
    frame.back() = frame_checksum(frame);
    AP_AOA_ALX::DecodedFrame decoded{};

    EXPECT_FALSE(AP_AOA_ALX::decode_frame(frame.data(), frame.size(), decoded));
}

TEST(AP_AOA_ALX_V5, RejectsTruncatedFrame)
{
    const Frame frame = make_protocol_frame(57);
    AP_AOA_ALX::DecodedFrame decoded{};

    EXPECT_FALSE(AP_AOA_ALX::decode_frame(frame.data(), frame.size() - 1, decoded));
}

TEST(AP_AOA_ALX_V5, RejectsWrongChecksum)
{
    Frame frame = make_protocol_frame(57);
    frame.back()++;
    AP_AOA_ALX::DecodedFrame decoded{};

    EXPECT_FALSE(AP_AOA_ALX::decode_frame(frame.data(), frame.size(), decoded));
}

TEST(AP_AOA_ALX_V5, ValidSampleCanBeConsumedExactlyOnceWithoutClearingConfidence)
{
    AP_AOA_ALX sensor;
    AP_AOA_ALX::DecodedFrame decoded{250, 57, 51, 213};
    ASSERT_TRUE(sensor.accept_decoded_frame(decoded, 100));

    float distance_m = 0.0f;
    float angle_deg = 0.0f;
    EXPECT_TRUE(sensor.get_raw_data(distance_m, angle_deg));
    EXPECT_FLOAT_EQ(2.5f, distance_m);
    EXPECT_FLOAT_EQ(57.0f, angle_deg);
    EXPECT_EQ(51, sensor.last_confidence());
    EXPECT_FALSE(sensor.get_raw_data(distance_m, angle_deg));
}

TEST(AP_AOA_ALX_V5, OnlyValidConfidentPositiveDistanceRefreshesFrameTime)
{
    AP_AOA_ALX sensor;
    EXPECT_TRUE(sensor.accept_decoded_frame({250, 57, 51, 213}, 100));
    EXPECT_EQ(20U, sensor.valid_frame_age_ms(120));

    EXPECT_FALSE(sensor.accept_decoded_frame({250, -4005, 0, 200}, 130));
    EXPECT_FALSE(sensor.accept_decoded_frame({0, 57, 51, 213}, 140));
    EXPECT_EQ(50U, sensor.valid_frame_age_ms(150));
}

AP_GTEST_MAIN()
