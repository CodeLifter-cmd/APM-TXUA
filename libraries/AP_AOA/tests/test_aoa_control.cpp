#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>

#include <AP_AOA/AP_AOAPID.h>
#include <AP_AOA/AP_AOADebugText.h>
#include <AP_AOA/AP_AOAFollowControl.h>

#include <cmath>
#include <cstring>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

TEST(AP_AOADebugText, FormatsCorrelatedMissionPlannerSnapshot)
{
    const AP_AOADebugText::Snapshot snapshot{
        123U,
        3.10f, -90.0f,
        3.08f, -88.0f,
        92.0f,
        25U, 4U, 3U, 12U, 2U,
        100.0f, 2700.0f, 1000.0f, 250.0f,
        0.4f, -1.2f,
    };
    AP_AOADebugText::Lines lines{};

    AP_AOADebugText::format(snapshot, lines);

    EXPECT_STREQ("A0 E00123 R3.1/-90 F3.1/-88 B92 D25", lines[0]);
    EXPECT_STREQ("A1 E00123 J4 G3 GR12 Q2", lines[1]);
    EXPECT_STREQ("A2 E00123 T100 S2700 L1000 R250 V0.4 Y-1.2", lines[2]);
}

TEST(AP_AOADebugText, KeepsWorstCaseLinesWithinStatustextPayload)
{
    const AP_AOADebugText::Snapshot snapshot{
        UINT32_MAX,
        20.0f, -180.0f,
        20.0f, -180.0f,
        -180.0f,
        1000U, UINT8_MAX, UINT8_MAX, UINT32_MAX, UINT32_MAX,
        100.0f, -4500.0f, -1000.0f, -1000.0f,
        99.9f, -999.9f,
    };
    AP_AOADebugText::Lines lines{};

    AP_AOADebugText::format(snapshot, lines);

    for (const auto &line : lines) {
        EXPECT_LT(strlen(line), AP_AOADebugText::TEXT_LENGTH);
    }
    EXPECT_EQ('\0', lines[AP_AOADebugText::LINE_COUNT - 1]
                           [AP_AOADebugText::TEXT_LENGTH - 1]);
}

TEST(AP_AOAPID, ResetPreventsFirstSampleDerivativeSpike)
{
    AP_AOAPID pid;
    pid.set_gains(0.0f, 0.0f, 1.0f, 1.0f);
    pid.reset();
    EXPECT_FLOAT_EQ(0.0f, pid.get_pid(100.0f, 0.1f));
}

TEST(AP_AOAPID, InvalidDtDoesNotChangeControllerState)
{
    AP_AOAPID pid;
    pid.set_gains(1.0f, 0.0f, 1.0f, 1.0f);
    pid.reset();
    EXPECT_FLOAT_EQ(0.0f, pid.get_pid(1.0f, NAN));
    EXPECT_FLOAT_EQ(1.0f, pid.get_pid(1.0f, 0.1f));
}

TEST(AP_AOAPID, POnlyPreservesDistanceAndSteeringSigns)
{
    AP_AOAPID pid;
    pid.set_gains(1.0f, 0.0f, 0.0f, 1.0f);
    pid.reset();
    EXPECT_GT(pid.get_pid(2.0f, 0.1f), 0.0f);
    EXPECT_GT(pid.get_pid(63.0f, 0.1f), 0.0f);
    EXPECT_LT(pid.get_pid(-111.0f, 0.1f), 0.0f);
}

TEST(AP_AOAPID, DerivativeFilterHasSameResponseOverOneHundredMilliseconds)
{
    AP_AOAPID pid_10hz;
    AP_AOAPID pid_40hz;
    pid_10hz.set_gains(0.0f, 0.0f, 1.0f, 100.0f);
    pid_40hz.set_gains(0.0f, 0.0f, 1.0f, 100.0f);

    EXPECT_FLOAT_EQ(0.0f, pid_10hz.get_pid(0.0f, 0.1f));
    EXPECT_FLOAT_EQ(0.0f, pid_40hz.get_pid(0.0f, 0.025f));

    const float output_10hz = pid_10hz.get_pid(1.0f, 0.1f);
    float output_40hz = 0.0f;
    for (uint8_t i = 1; i <= 4; i++) {
        output_40hz = pid_40hz.get_pid(0.25f * i, 0.025f);
    }
    EXPECT_NEAR(output_10hz, output_40hz, 0.001f);
}

TEST(AP_AOAPID, ReportsHandCalculatedPidTerms)
{
    AP_AOAPID pid;
    pid.set_gains(2.0f, 1.0f, 0.5f, 10.0f);

    EXPECT_FLOAT_EQ(4.2f, pid.get_pid(2.0f, 0.1f));
    const AP_AOAPID::Info first = pid.info();
    EXPECT_FLOAT_EQ(2.0f, first.error);
    EXPECT_FLOAT_EQ(0.1f, first.dt);
    EXPECT_FLOAT_EQ(4.0f, first.p);
    EXPECT_FLOAT_EQ(0.2f, first.i);
    EXPECT_FLOAT_EQ(0.0f, first.d);
    EXPECT_FLOAT_EQ(4.2f, first.output);

    EXPECT_FLOAT_EQ(7.5f, pid.get_pid(3.0f, 0.1f));
    const AP_AOAPID::Info second = pid.info();
    EXPECT_FLOAT_EQ(6.0f, second.p);
    EXPECT_FLOAT_EQ(0.5f, second.i);
    EXPECT_FLOAT_EQ(1.0f, second.d);
    EXPECT_FLOAT_EQ(7.5f, second.output);
}

TEST(AP_AOAFollowControl, UsesTimeBetweenAcceptedSamples)
{
    AP_AOAFollowControl control;
    control.configure(0.0f, 0.0f, 1.0f, 1.0f,
                      0.0f, 0.0f, 0.0f, 1.0f,
                      1.0f, 1.0f, 1.0f);

    EXPECT_TRUE(control.accept_sample(2.0f, 0.0f, 100U));
    EXPECT_FLOAT_EQ(0.0f, control.output().throttle);
    EXPECT_TRUE(control.accept_sample(3.0f, 0.0f, 600U));
    EXPECT_FLOAT_EQ(40.0f, control.output().throttle);
}

TEST(AP_AOAFollowControl, CoversDistanceAndSteeringDirections)
{
    AP_AOAFollowControl control;
    control.configure(1.0f, 0.0f, 0.0f, 1.0f,
                      1.0f, 0.0f, 0.0f, 45.0f,
                      1.0f, 1.0f, 1.0f);

    EXPECT_TRUE(control.accept_sample(2.0f, 10.0f, 100U));
    EXPECT_GT(control.output().throttle, 0.0f);
    EXPECT_GT(control.output().steering, 0.0f);

    EXPECT_TRUE(control.accept_sample(2.0f, -10.0f, 200U));
    EXPECT_LT(control.output().steering, 0.0f);

    EXPECT_FALSE(control.accept_sample(1.0f, 10.0f, 300U));
    EXPECT_FLOAT_EQ(0.0f, control.output().throttle);
    EXPECT_FLOAT_EQ(0.0f, control.output().steering);
}

TEST(AP_AOAFollowControl, HoldsThenStopsAfterDataTimeout)
{
    AP_AOAFollowControl control;
    control.configure(1.0f, 0.0f, 0.0f, 1.0f,
                      1.0f, 0.0f, 0.0f, 45.0f,
                      1.0f, 1.0f, 1.0f);

    ASSERT_TRUE(control.accept_sample(2.0f, 10.0f, 100U));
    const AP_AOAFollowControl::Output held = control.output();
    EXPECT_FALSE(control.check_timeout(1100U, 1000U));
    EXPECT_FLOAT_EQ(held.throttle, control.output().throttle);
    EXPECT_TRUE(control.check_timeout(1101U, 1000U));
    EXPECT_FLOAT_EQ(0.0f, control.output().throttle);
    EXPECT_FLOAT_EQ(0.0f, control.output().steering);
}

TEST(AP_AOAFollowControl, ReinitialisesPidAfterLongSampleGap)
{
    AP_AOAFollowControl control;
    control.configure(0.0f, 1.0f, 0.0f, 1.0f,
                      0.0f, 0.0f, 0.0f, 1.0f,
                      1.0f, 1.0f, 1.0f);

    ASSERT_TRUE(control.accept_sample(2.0f, 0.0f, 100U));
    ASSERT_TRUE(control.accept_sample(2.0f, 0.0f, 1101U));
    EXPECT_LT(control.output().throttle, 0.01f);
}

TEST(AP_AOAFollowControl, EntersTooCloseAtTargetDistance)
{
    AP_AOAFollowControl control;
    control.configure(1, 0, 0, 1, 1, 0, 0, 45, 1, 1, 1);
    EXPECT_FALSE(control.accept_sample(1.0f, 20.0f, 100U));
    EXPECT_TRUE(control.too_close());
    EXPECT_FLOAT_EQ(0, control.output().throttle);
    EXPECT_FLOAT_EQ(0, control.output().steering);
}

TEST(AP_AOAFollowControl, HoldsStopInsideHysteresisWindow)
{
    AP_AOAFollowControl control;
    control.configure(1, 0, 0, 1, 1, 0, 0, 45, 1, 1, 1);
    control.set_distance_window(1.0f, 0.3f);
    ASSERT_FALSE(control.accept_sample(0.9f, 0, 100U));
    EXPECT_FALSE(control.accept_sample(1.1f, 0, 200U));
    EXPECT_FALSE(control.accept_sample(1.29f, 0, 300U));
    EXPECT_TRUE(control.too_close());
}

TEST(AP_AOAFollowControl, RequiresTwentyRecoveryFramesAtFortyHertz)
{
    AP_AOAFollowControl control;
    control.configure(1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1);
    control.set_distance_window(1.0f, 0.3f);
    ASSERT_FALSE(control.accept_sample(1.0f, 0, 100U));
    for (uint8_t i = 1; i <= 19; i++) {
        EXPECT_FALSE(control.accept_sample(1.3f, 0, 100U + i * 25U));
        EXPECT_TRUE(control.too_close());
        EXPECT_EQ(i, control.resume_frame_count());
    }
    EXPECT_FALSE(control.accept_sample(1.3f, 0, 600U));
    EXPECT_FALSE(control.too_close());
    EXPECT_FLOAT_EQ(0, control.output().throttle);
    EXPECT_TRUE(control.accept_sample(1.3f, 0, 625U));
    EXPECT_GT(control.output().throttle, 0);
}

TEST(AP_AOAFollowControl, BelowRestartThresholdResetsRecoveryCount)
{
    AP_AOAFollowControl control;
    control.configure(1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1);
    control.set_distance_window(1.0f, 0.3f);
    ASSERT_FALSE(control.accept_sample(0.9f, 0, 100U));
    control.accept_sample(1.31f, 0, 200U);
    control.accept_sample(1.32f, 0, 300U);
    control.accept_sample(1.29f, 0, 400U);
    EXPECT_EQ(0, control.resume_frame_count());
    for (uint32_t t = 500; t <= 800; t += 100) {
        EXPECT_FALSE(control.accept_sample(1.35f, 0, t));
    }
    EXPECT_TRUE(control.too_close());
}

TEST(AP_AOAFollowControl, SingleFarOutlierCannotRestart)
{
    AP_AOAFollowControl control;
    control.configure(1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1);
    control.set_distance_window(1.0f, 0.3f);
    control.accept_sample(0.9f, 0, 100U);
    EXPECT_FALSE(control.accept_sample(2.5f, 0, 200U));
    EXPECT_TRUE(control.too_close());
    EXPECT_EQ(1, control.resume_frame_count());
}

TEST(AP_AOAFollowControl, TargetDistanceChangeAppliesImmediately)
{
    AP_AOAFollowControl control;
    control.configure(1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1);
    EXPECT_TRUE(control.accept_sample(1.1f, 0, 100U));
    EXPECT_TRUE(control.set_distance_window(1.2f, 0.3f));
    EXPECT_FALSE(control.accept_sample(1.1f, 0, 200U));
    EXPECT_TRUE(control.too_close());
}

TEST(AP_AOAFollowControl, SafeDistanceChangeStopsForOneSample)
{
    AP_AOAFollowControl control;
    control.configure(1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1);
    ASSERT_TRUE(control.accept_sample(1.5f, 0, 100U));
    ASSERT_TRUE(control.set_distance_window(0.8f, 0.3f));
    EXPECT_FALSE(control.accept_sample(1.5f, 0, 200U));
    EXPECT_FALSE(control.too_close());
    EXPECT_TRUE(control.accept_sample(1.5f, 0, 300U));
}

TEST(AP_AOAFollowControl, UnchangedDistanceWindowDoesNotResetControl)
{
    AP_AOAFollowControl control;
    control.configure(0, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1);
    ASSERT_TRUE(control.accept_sample(2.0f, 0, 100U));
    ASSERT_TRUE(control.accept_sample(2.0f, 0, 200U));
    const float before = control.output().throttle;
    EXPECT_FALSE(control.set_distance_window(1.0f, 0.3f));
    ASSERT_TRUE(control.accept_sample(2.0f, 0, 300U));
    EXPECT_GT(control.output().throttle, before);
}

TEST(AP_AOAFollowControl, ChangingTargetDoesNotReleaseLatchedStop)
{
    AP_AOAFollowControl control;
    control.configure(1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1);
    control.set_distance_window(1.0f, 0.3f);
    control.accept_sample(0.9f, 0, 100U);
    EXPECT_TRUE(control.set_distance_window(0.8f, 0.3f));
    for (uint8_t i = 1; i <= 4; i++) {
        EXPECT_FALSE(control.accept_sample(1.1f, 0, 100U + i * 100U));
    }
    EXPECT_TRUE(control.too_close());
}

TEST(AP_AOAFollowControl, TimeoutWhileTooCloseKeepsLatch)
{
    AP_AOAFollowControl control;
    control.configure(1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1);
    control.set_distance_window(1.0f, 0.3f);
    control.accept_sample(0.9f, 0, 100U);
    control.accept_sample(1.3f, 0, 200U);
    EXPECT_TRUE(control.check_timeout(1201U, 1000U));
    EXPECT_TRUE(control.too_close());
    EXPECT_EQ(0, control.resume_frame_count());
}

TEST(AP_AOAFollowControl, FirstSampleAfterRecoveryHasNoIntegralOrDerivativeSpike)
{
    AP_AOAFollowControl control;
    control.configure(0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1);
    control.set_distance_window(1.0f, 0.3f);
    control.accept_sample(0.9f, 0, 100U);
    for (uint8_t i = 1; i <= 20; i++) {
        EXPECT_FALSE(control.accept_sample(1.3f, 0, 100U + i * 25U));
    }
    ASSERT_TRUE(control.accept_sample(1.3f, 0, 625U));
    EXPECT_LT(control.output().throttle, 0.01f);
}

TEST(AP_AOAFollowControl, LongGapAfterNineteenFramesRestartsRecoveryCount)
{
    AP_AOAFollowControl control;
    control.configure(1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1);
    control.set_distance_window(1.0f, 0.3f);
    control.accept_sample(0.9f, 0, 100U);
    for (uint8_t i = 1; i <= 19; i++) {
        ASSERT_FALSE(control.accept_sample(1.3f, 0, 100U + i * 25U));
    }
    ASSERT_EQ(19, control.resume_frame_count());

    EXPECT_FALSE(control.accept_sample(1.3f, 0, 1576U));
    EXPECT_TRUE(control.too_close());
    EXPECT_EQ(1, control.resume_frame_count());
}

TEST(AP_AOAFollowControl, RejectedSampleClearsRecoveryWithoutReleasingLatch)
{
    AP_AOAFollowControl control;
    control.configure(1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1);
    control.set_distance_window(1.0f, 0.3f);
    control.accept_sample(0.9f, 0, 100U);
    control.accept_sample(1.3f, 0, 200U);
    control.accept_sample(1.3f, 0, 300U);
    ASSERT_EQ(2, control.resume_frame_count());

    control.reject_sample();
    EXPECT_TRUE(control.too_close());
    EXPECT_EQ(0, control.resume_frame_count());
    EXPECT_FLOAT_EQ(0, control.output().throttle);
    EXPECT_FLOAT_EQ(0, control.output().steering);
}

TEST(AP_AOAFollowControl, AngleDeadzoneSuppressesSmallSteering)
{
    AP_AOAFollowControl control;
    control.configure(1, 0, 0, 1, 1, 0, 0, 45, 1, 1, 1);
    control.set_steering_smoothing(3.0f, 0.0f);
    ASSERT_TRUE(control.accept_sample(2.0f, 3.0f, 100U));
    EXPECT_FLOAT_EQ(0.0f, control.output().steering);
    ASSERT_TRUE(control.accept_sample(2.0f, 3.1f, 200U));
    EXPECT_GT(control.output().steering, 0.0f);
}

TEST(AP_AOAFollowControl, SteeringRateLimitsSignReversal)
{
    AP_AOAFollowControl control;
    control.configure(1, 0, 0, 1, 1, 0, 0, 45, 1, 1, 1);
    control.set_steering_smoothing(0.0f, 18000.0f);
    ASSERT_TRUE(control.accept_sample(2.0f, 45.0f, 100U));
    EXPECT_NEAR(0.018f, control.output().steering, 0.001f);
    ASSERT_TRUE(control.accept_sample(2.0f, -45.0f, 150U));
    EXPECT_NEAR(-899.982f, control.output().steering, 0.01f);
    EXPECT_GT(control.output().steering, -4500.0f);
}

TEST(AP_AOAFollowControl, SafetyStopBypassesSteeringRateLimit)
{
    AP_AOAFollowControl control;
    control.configure(1, 0, 0, 1, 1, 0, 0, 45, 1, 1, 1);
    control.set_steering_smoothing(0.0f, 18000.0f);
    ASSERT_TRUE(control.accept_sample(2.0f, 45.0f, 100U));
    EXPECT_FALSE(control.accept_sample(1.0f, 45.0f, 150U));
    EXPECT_FLOAT_EQ(0.0f, control.output().steering);
}

TEST(AP_AOAFollowControl, EnteringAngleDeadzoneImmediatelyZerosSteering)
{
    AP_AOAFollowControl control;
    control.configure(1, 0, 0, 1, 1, 0.5f, 0, 45, 1, 1, 1);
    control.set_steering_smoothing(3.0f, 18000.0f);

    for (uint32_t timestamp_ms = 100; timestamp_ms <= 400; timestamp_ms += 50) {
        ASSERT_TRUE(control.accept_sample(2.0f, 45.0f, timestamp_ms));
    }
    ASSERT_GT(control.output().steering, 1000.0f);

    ASSERT_TRUE(control.accept_sample(2.0f, 2.0f, 450U));
    EXPECT_FLOAT_EQ(0.0f, control.output().steering);
    EXPECT_GT(control.output().throttle, 0.0f);
}

TEST(AP_AOAFollowControl, KeepsDistanceThrottleAtLargeTargetAngles)
{
    AP_AOAFollowControl control;
    control.configure(1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1);

    ASSERT_TRUE(control.accept_sample(2.0f, 0.0f, 100U));
    EXPECT_FLOAT_EQ(100.0f, control.output().throttle);
    ASSERT_TRUE(control.accept_sample(2.0f, 90.0f, 200U));
    EXPECT_FLOAT_EQ(100.0f, control.output().throttle);
    ASSERT_TRUE(control.accept_sample(2.0f, -179.0f, 300U));
    EXPECT_FLOAT_EQ(100.0f, control.output().throttle);
}

TEST(AP_AOAFollowControl, RearTargetSteeringUsesCurrentAngleSign)
{
    AP_AOAFollowControl control;
    control.configure(1, 0, 0, 1, 1, 0, 0, 45, 1, 1, 1);
    control.set_steering_smoothing(0.0f, 0.0f);

    ASSERT_TRUE(control.accept_sample(2.0f, 179.0f, 100U));
    EXPECT_GT(control.output().steering, 0.0f);
    ASSERT_TRUE(control.accept_sample(2.0f, -179.0f, 200U));
    EXPECT_LT(control.output().steering, 0.0f);
}

TEST(AP_AOAFollowControl, ReportsAcceptedSampleAndPidDiagnostics)
{
    AP_AOAFollowControl control;
    control.configure(1, 0, 0, 1, 1, 0, 0, 45, 1, 1, 1);

    ASSERT_TRUE(control.accept_sample(2.0f, 10.0f, 100U));
    const AP_AOAFollowControl::Diagnostics &first = control.diagnostics();
    EXPECT_TRUE(first.sample_valid);
    EXPECT_TRUE(first.control_accepted);
    EXPECT_FALSE(first.angle_deadzone);
    EXPECT_FLOAT_EQ(2.0f, first.distance_m);
    EXPECT_FLOAT_EQ(10.0f, first.body_angle_deg);
    EXPECT_FLOAT_EQ(1.0f, first.distance_error);
    EXPECT_FLOAT_EQ(10.0f, first.angle_error);
    EXPECT_FLOAT_EQ(0.0f, first.dt);
    EXPECT_FLOAT_EQ(1.0f, control.distance_pid_info().p);
    EXPECT_FLOAT_EQ(10.0f, control.angle_pid_info().p);

    ASSERT_TRUE(control.accept_sample(2.0f, 12.0f, 125U));
    EXPECT_FLOAT_EQ(0.025f, control.diagnostics().dt);
}

TEST(AP_AOAFollowControl, ReportsSafetyRejectedSampleState)
{
    AP_AOAFollowControl control;
    control.configure(1, 0, 0, 1, 1, 0, 0, 45, 1, 1, 1);

    ASSERT_FALSE(control.accept_sample(0.8f, -5.0f, 100U));
    const AP_AOAFollowControl::Diagnostics &diagnostics = control.diagnostics();
    EXPECT_TRUE(diagnostics.sample_valid);
    EXPECT_FALSE(diagnostics.control_accepted);
    EXPECT_TRUE(control.too_close());
    EXPECT_FLOAT_EQ(0.8f, diagnostics.distance_m);
    EXPECT_FLOAT_EQ(-5.0f, diagnostics.body_angle_deg);
    EXPECT_FLOAT_EQ(-0.2f, diagnostics.distance_error);
    EXPECT_FLOAT_EQ(-5.0f, diagnostics.angle_error);
}

AP_GTEST_MAIN()
