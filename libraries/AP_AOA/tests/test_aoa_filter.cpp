#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>

#include <AP_AOA/AP_AOAFilter.h>

#include <cmath>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

TEST(AP_AOAFilter, PassThroughKeepsAngleAndDistanceIndependent)
{
    AOAKalmanFilter filter;
    ASSERT_TRUE(filter.update(123.0f, 2.5f));
    EXPECT_FLOAT_EQ(123.0f, filter.get_angle());
    EXPECT_FLOAT_EQ(2.5f, filter.get_distance());
}

TEST(AP_AOAFilter, BodyAngleUsesSixtyDegreeOffsetAndWraps)
{
    struct Case { float sensor; float body; };
    const Case cases[] = {
        {57, -3}, {123, 63}, {129, 69}, {194, 134}, {204, 144},
        {219, 159}, {221, 161}, {241, -179}, {256, -164}, {265, -155},
        {-51, -111}, {-54, -114}, {-73, -133}, {-87, -147},
    };
    AOAKalmanFilter filter;
    for (const auto &test_case : cases) {
        ASSERT_TRUE(filter.update(test_case.sensor, 2.0f));
        EXPECT_FLOAT_EQ(test_case.body, filter.get_body_angle(60.0f));
    }
}

TEST(AP_AOAFilter, CircularLowPassTakesShortestPathAcrossBoundary)
{
    AOAKalmanFilter filter;
    filter.set_time_constant(1.0f);
    ASSERT_TRUE(filter.update(161.0f, 2.0f, 1.0f));
    ASSERT_TRUE(filter.update(-179.0f, 2.0f, 1.0f));
    EXPECT_NEAR(171.0f, filter.get_angle(), 0.001f);
    ASSERT_TRUE(filter.update(-164.0f, 2.0f, 1.0f));
    EXPECT_NEAR(-176.5f, filter.get_angle(), 0.001f);
}

TEST(AP_AOAFilter, InvalidMeasurementsDoNotPolluteState)
{
    AOAKalmanFilter filter;
    ASSERT_TRUE(filter.update(57.0f, 2.0f));
    EXPECT_FALSE(filter.update(NAN, 3.0f));
    EXPECT_FALSE(filter.update(90.0f, INFINITY));
    EXPECT_FALSE(filter.update(90.0f, 0.0f));
    EXPECT_FLOAT_EQ(57.0f, filter.get_angle());
    EXPECT_FLOAT_EQ(2.0f, filter.get_distance());
}

TEST(AP_AOAFilter, RequiresThreeFramesToConfirmLargeAngleJump)
{
    AOAKalmanFilter filter;
    filter.set_angle_jump_limit(60.0f);
    ASSERT_TRUE(filter.update(0.0f, 2.0f, 0.05f));
    ASSERT_TRUE(filter.update(100.0f, 2.0f, 0.05f));
    EXPECT_FLOAT_EQ(0.0f, filter.get_angle());
    ASSERT_TRUE(filter.update(102.0f, 2.0f, 0.05f));
    EXPECT_FLOAT_EQ(0.0f, filter.get_angle());
    ASSERT_TRUE(filter.update(101.0f, 2.0f, 0.05f));
    EXPECT_FLOAT_EQ(101.0f, filter.get_angle());
}

TEST(AP_AOAFilter, NormalAngleChangesRemainImmediate)
{
    AOAKalmanFilter filter;
    filter.set_angle_jump_limit(60.0f);
    ASSERT_TRUE(filter.update(0.0f, 2.0f));
    ASSERT_TRUE(filter.update(45.0f, 2.0f));
    EXPECT_FLOAT_EQ(45.0f, filter.get_angle());
}

AP_GTEST_MAIN()
