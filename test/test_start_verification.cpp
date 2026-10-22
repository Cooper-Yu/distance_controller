/** @file @brief Acceptance and rejection of stopped scan batches. */
#include <gtest/gtest.h>

#include <limits>

#include "distance_controller/start_adjustment.hpp"
#include "distance_controller/start_diagnostics.hpp"
#include "distance_controller/start_verification.hpp"
using distance_controller::StartSample;
using distance_controller::summarize_start;

TEST(StartVerification, MedianRejectsThreeJointOutliers)
{
  std::vector<StartSample> s(15, {.26, .28, 0});
  s[0] = {.31, .28, 0};
  s[1] = {.26, .33, 0};
  s[2] = {.26, .28, .1};
  const auto v = summarize_start(s);
  EXPECT_TRUE(v.stable);
  EXPECT_EQ(v.retained, 12u);
  EXPECT_DOUBLE_EQ(v.median[0], .26);
  EXPECT_DOUBLE_EQ(v.median[1], .28);
}

TEST(StartVerification, RejectsInsufficientAndNonfiniteEvidence)
{
  std::vector<StartSample> s(14, {.26, .28, 0});
  EXPECT_FALSE(summarize_start(s).stable);
  s.push_back({.26, .28, 0});
  for (int i = 0; i < 4; ++i) s[i][2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(summarize_start(s).stable);
}

TEST(StartVerification, SplitWallAndSmallDriftAreNotStable)
{
  std::vector<StartSample> s(15, {.26, .28, 0});
  for (int i = 0; i < 7; ++i) s[i][0] += .03;
  EXPECT_FALSE(summarize_start(s).stable);
  for (int i = 0; i < 15; ++i) s[i][0] = .26 + .001 * i;
  EXPECT_FALSE(summarize_start(s).stable);
}

TEST(StartVerification, StableDoesNotMeanOnTarget)
{
  std::vector<StartSample> s(15, {.268, .28, .015});
  const auto v = summarize_start(s);
  EXPECT_TRUE(v.stable);
  EXPECT_GT(std::abs(v.median[0] - .26), .005);
  EXPECT_GT(std::abs(v.median[2]), .008726646);
}

TEST(StartDiagnostics, CountsBlockingEpisodesAndDiscardedSamplesNotEveryTimerTick)
{
  distance_controller::StartDiagnostics d;
  d.observe_stop(d.LinearSpeed, 7);
  for (unsigned tick = 0; tick < 40; ++tick) d.observe_stop(d.LinearSpeed, 0);
  EXPECT_EQ(d.reset_events, 1u);
  EXPECT_EQ(d.linear_resets, 1u);
  EXPECT_EQ(d.blocked_ticks, 41u);
  EXPECT_EQ(d.discarded, 7u);
  d.observe_stop(0, 0);
  d.observe_stop(d.AngularSpeed | d.AnchorMotion, 3);
  EXPECT_EQ(d.reset_events, 2u);
  EXPECT_EQ(d.angular_resets, 1u);
  EXPECT_EQ(d.anchor_resets, 1u);
  EXPECT_EQ(d.discarded, 10u);
}

TEST(StartDiagnostics, RemembersActualUnstableBatchWithoutChangingSummary)
{
  std::vector<StartSample> samples(15, {.26, .28, 0});
  for (unsigned i = 0; i < 4; ++i) samples[i][0] += .010408;
  const auto summary = summarize_start(samples);
  distance_controller::StartDiagnostics d;
  d.observe_batch(summary);
  EXPECT_EQ(d.batches, 1u);
  EXPECT_EQ(d.last_batch.retained, 15u);
  EXPECT_FALSE(d.last_batch.stable);
  EXPECT_DOUBLE_EQ(d.last_batch.spread[0], summary.spread[0]);
  EXPECT_EQ(samples.size(), 15u);
  d = {};
  EXPECT_EQ(d.batches, 0u);
  EXPECT_EQ(d.reset_events, 0u);
}

TEST(StartVerification, RollingWindowFindsStableOffsetAcrossRejectedBatchBoundary)
{
  std::vector<StartSample> rolling, first;
  bool found_stable = false;
  for (unsigned i = 0; i < 30; ++i) {
    StartSample sample{.261149, i % 5 == 0 ? .2845 : .28892674018, -.00673455779};
    if (i < 8 || i >= 24) sample[0] += .012;
    if (i < 15) first.push_back(sample);
    distance_controller::append_start_sample(rolling, sample);
    const auto summary = summarize_start(rolling);
    if (summary.stable) {
      found_stable = true;
      EXPECT_GT(summary.median[1] - .28, .005);
      const auto delta = distance_controller::start_adjustment(summary, .26, .28, .005);
      ASSERT_TRUE(delta);
      EXPECT_NEAR((*delta)[0], -.005, 1e-12);
      EXPECT_DOUBLE_EQ((*delta)[1], 0.0);
    }
  }
  EXPECT_FALSE(summarize_start(first).stable);
  EXPECT_TRUE(found_stable);
  EXPECT_EQ(rolling.size(), 15u);
}

TEST(StartAdjustment, DirectionDeadbandAndCombinedNormAreBounded)
{
  for (double sign : {-1., 1.}) {
    auto summary =
      summarize_start(std::vector<StartSample>(15, {.26 + sign * .019, .28 + sign * .019, 0}));
    const auto delta = distance_controller::start_adjustment(summary, .26, .28, .005);
    ASSERT_TRUE(delta);
    EXPECT_NEAR(std::hypot((*delta)[0], (*delta)[1]), .005, 1e-12);
    EXPECT_LT(sign * (*delta)[0], 0);
    EXPECT_LT(sign * (*delta)[1], 0);
  }
  auto summary = summarize_start(std::vector<StartSample>(15, {.26, .301, 0}));
  EXPECT_FALSE(distance_controller::start_adjustment(summary, .26, .28, .005));
  summary.stable = false;
  EXPECT_FALSE(distance_controller::start_adjustment(summary, .26, .28, .005));
}

TEST(StartAdjustment, FrozenOdomTargetTracksRotatedBodyWithNormCapAndZeroArrival)
{
  const double half_pi = 1.5707963267948966;
  distance_controller::StartCorrection step{1, 2, half_pi, 0, -.009, "odom", "base_link"};
  const auto initial =
    distance_controller::correction_feedback(step, 1, 2, half_pi, 0, .008726646, 0, .02);
  EXPECT_FALSE(initial.fault);
  EXPECT_NEAR(initial.vx, -.004, 1e-12);
  EXPECT_NEAR(initial.vy, 0, 1e-12);
  const auto done =
    distance_controller::correction_feedback(step, 1, 1.9912, half_pi, 0, .008726646, 2, .008);
  EXPECT_TRUE(done.arrived);
  EXPECT_FALSE(done.remeasure);
  EXPECT_DOUBLE_EQ(done.vx, 0);
  EXPECT_DOUBLE_EQ(done.vy, 0);
  EXPECT_DOUBLE_EQ(done.wz, 0);
}

TEST(StartAdjustment, FaultBoundsAlwaysReturnZero)
{
  distance_controller::StartCorrection step{0, 0, 0, -.009, 0, "odom", "base_link"};
  const std::array<std::array<double, 5>, 6> cases{
    {{0, 0, 0, 0, 4.01},
     {-.013, 0, 0, 0, 1},
     {0, .0031, 0, 0, 1},
     {0, 0, .0088, 0, 1},
     {0, 0, 0, .009, 1},
     {std::numeric_limits<double>::quiet_NaN(), 0, 0, 0, 1}}};
  for (const auto & value : cases) {
    const auto result = distance_controller::correction_feedback(
      step, value[0], value[1], value[2], value[3], .008726646, value[4], .008);
    EXPECT_TRUE(result.fault);
    EXPECT_FALSE(result.arrived);
    EXPECT_FALSE(result.remeasure);
    EXPECT_DOUBLE_EQ(result.vx, 0);
    EXPECT_DOUBLE_EQ(result.vy, 0);
    EXPECT_DOUBLE_EQ(result.wz, 0);
  }
}

TEST(StartAdjustment, SmallOvershootStopsForRemeasurementInsteadOfReversing)
{
  const distance_controller::StartCorrection step{0, 0, 0, -.009, 0, "odom", "base_link"};
  const auto result =
    distance_controller::correction_feedback(step, -.0097, 0, 0, 0, .008726646, 2, .008);
  EXPECT_FALSE(result.fault);
  EXPECT_FALSE(result.arrived);
  EXPECT_TRUE(result.remeasure);
  EXPECT_DOUBLE_EQ(result.vx, 0.0);
  EXPECT_DOUBLE_EQ(result.wz, 0.0);
  EXPECT_DOUBLE_EQ(result.vy, 0.0);
  EXPECT_DOUBLE_EQ(step.dx, -.009);
}

TEST(StartAdjustment, HeadingHoldUsesInitializationReferenceAndShortestSignedRotation)
{
  const double pi = 3.141592653589793;
  for (const double anchor : {0.0, pi - .001, -pi + .001}) {
    const distance_controller::StartCorrection step{1, 2, anchor, .009, 0, "odom", "base_link"};
    for (const double drift : {-.004, 0.0, .004}) {
      const auto result = distance_controller::correction_feedback(
        step, 1, 2, std::remainder(anchor + drift, 2 * pi), drift, .008726646, 1, .02);
      ASSERT_FALSE(result.fault);
      EXPECT_NEAR(result.wz, -1.8 * drift, 1e-12);
      EXPECT_LE(std::abs(result.wz), .02);
      EXPECT_DOUBLE_EQ(step.yaw, anchor);
      EXPECT_DOUBLE_EQ(step.dx, .009);
    }
  }
}

TEST(StartAdjustment, HeadingHoldSuppressesSmallConstantYawBiasWithoutMovingTheTarget)
{
  const distance_controller::StartCorrection step{0, 0, 0, -.009, 0, "odom", "base_link"};
  for (const double bias : {-.0015, .0015}) {
    double x = 0, y = 0, yaw = bias > 0 ? .003 : -.003;
    bool arrived = false;
    for (int tick = 0; tick <= 200; ++tick) {
      const auto result = distance_controller::correction_feedback(
        step, x, y, yaw, yaw, .008726646, tick * .02, .008);
      ASSERT_FALSE(result.fault);
      ASSERT_LE(std::abs(yaw), .005);
      EXPECT_LE(std::hypot(result.vx, result.vy), .004000001);
      EXPECT_LE(std::abs(result.wz), .02);
      if (result.arrived || result.remeasure) {
        EXPECT_DOUBLE_EQ(result.vx, 0);
        EXPECT_DOUBLE_EQ(result.vy, 0);
        EXPECT_DOUBLE_EQ(result.wz, 0);
        arrived = true;
        break;
      }
      x += (std::cos(yaw) * result.vx - std::sin(yaw) * result.vy) * .02;
      y += (std::sin(yaw) * result.vx + std::cos(yaw) * result.vy) * .02;
      yaw += (result.wz + bias) * .02;
    }
    EXPECT_TRUE(arrived);
    EXPECT_LT(std::abs(yaw), .003);
    EXPECT_NEAR(x, -.0025, .00008);
    EXPECT_NEAR(y, 0, 1e-12);
    EXPECT_DOUBLE_EQ(step.dx, -.009);
    EXPECT_DOUBLE_EQ(step.dy, 0);
    EXPECT_DOUBLE_EQ(step.yaw, 0);
  }
}

TEST(StartAdjustment, ThreeRefinementsResolveObservedLateralErrorAtReducedSpeed)
{
  for (const double sign : {-1.0, 1.0}) {
    double x = 0, y = 0;
    unsigned attempts = 0;
    while (std::abs(sign * .009501 + y) > .005 && attempts < 3) {
      const auto summary =
        summarize_start(std::vector<StartSample>(15, {.26 + sign * .009501 + y, .28 + x, 0}));
      const auto delta = distance_controller::start_adjustment(summary, .26, .28, .005);
      ASSERT_TRUE(delta);
      ASSERT_LE(std::hypot((*delta)[0], (*delta)[1]), .005000000001);
      const distance_controller::StartCorrection step{x,           y,      0,          (*delta)[0],
                                                      (*delta)[1], "odom", "base_link"};
      bool arrived = false;
      for (int tick = 0; tick <= 200; ++tick) {
        const auto feedback =
          distance_controller::correction_feedback(step, x, y, 0, 0, .008726646, tick * .02, .008);
        ASSERT_FALSE(feedback.fault);
        EXPECT_LE(std::hypot(feedback.vx, feedback.vy), .004000000001);
        if (feedback.arrived || feedback.remeasure) {
          EXPECT_DOUBLE_EQ(feedback.vx, 0);
          EXPECT_DOUBLE_EQ(feedback.vy, 0);
          EXPECT_DOUBLE_EQ(feedback.wz, 0);
          arrived = true;
          break;
        }
        x += feedback.vx * .02;
        y += feedback.vy * .02;
      }
      ASSERT_TRUE(arrived);
      EXPECT_NEAR(std::hypot(x - step.x, y - step.y), .0025, .00008);
      EXPECT_DOUBLE_EQ(step.dx, (*delta)[0]);
      EXPECT_DOUBLE_EQ(step.dy, (*delta)[1]);
      ++attempts;
    }
    EXPECT_EQ(attempts, 2u);
    EXPECT_LE(attempts, 3u);
    EXPECT_LE(std::abs(sign * .009501 + y), .005);
    EXPECT_DOUBLE_EQ(x, 0);
  }
}

TEST(StartAdjustment, SignedForwardBackwardAndDiagonalBrakeAtProjectedThreshold)
{
  const double diagonal = std::sqrt(.5);
  for (const std::array<double, 2> direction :
       {std::array<double, 2>{1, 0}, {-1, 0}, {diagonal, diagonal}, {-diagonal, -diagonal}}) {
    for (const double length : {.005, .002}) {
      const double threshold = std::min(.0025, length * .5);
      const distance_controller::StartCorrection step{
        1, 2, .3, length * direction[0], length * direction[1], "odom", "base_link"};
      const auto before = distance_controller::correction_feedback(
        step, 1 + (threshold - .00001) * direction[0], 2 + (threshold - .00001) * direction[1], .3,
        0, .008726646, .5, .008);
      ASSERT_FALSE(before.fault);
      EXPECT_FALSE(before.arrived);
      EXPECT_FALSE(before.remeasure);
      EXPECT_GT(std::hypot(before.vx, before.vy), 0);
      const auto brake = distance_controller::correction_feedback(
        step, 1 + (threshold + .00001) * direction[0], 2 + (threshold + .00001) * direction[1],
        .302, 0, .008726646, 1, .008);
      ASSERT_FALSE(brake.fault);
      EXPECT_FALSE(brake.arrived);
      EXPECT_TRUE(brake.remeasure);
      EXPECT_DOUBLE_EQ(brake.vx, 0);
      EXPECT_DOUBLE_EQ(brake.vy, 0);
      EXPECT_DOUBLE_EQ(brake.wz, 0);
      EXPECT_DOUBLE_EQ(step.dx, length * direction[0]);
      EXPECT_DOUBLE_EQ(step.dy, length * direction[1]);
    }
  }
}

TEST(StartAdjustment, FaultGuardsTakePriorityAfterEarlyBrakeThreshold)
{
  const distance_controller::StartCorrection step{0, 0, 0, .005, 0, "odom", "base_link"};
  const std::array<std::array<double, 5>, 6> cases{
    {{.003, 0, 0, 0, 4.01},
     {.013, 0, 0, 0, 1},
     {.003, .0031, 0, 0, 1},
     {.003, 0, .0088, 0, 1},
     {.003, 0, 0, .009, 1},
     {.003, 0, std::numeric_limits<double>::quiet_NaN(), 0, 1}}};
  for (const auto & value : cases) {
    const auto result = distance_controller::correction_feedback(
      step, value[0], value[1], value[2], value[3], .008726646, value[4], .008);
    EXPECT_TRUE(result.fault);
    EXPECT_FALSE(result.arrived);
    EXPECT_FALSE(result.remeasure);
    EXPECT_DOUBLE_EQ(result.vx, 0);
    EXPECT_DOUBLE_EQ(result.vy, 0);
    EXPECT_DOUBLE_EQ(result.wz, 0);
  }
}

TEST(StartAdjustment, FieldPoseSampleRequestsZeroAndFreshMeasurementBeforeTargetArrival)
{
  // Recorded filtered-odom sample; this does not model mechanical coast or filter latency.
  const distance_controller::StartCorrection step{.390251, 2.026792, -1.557447,  .004999,
                                                  .000067, "odom",   "base_link"};
  const double x = .393780, y = 2.028174, yaw = -1.55493;
  EXPECT_GT(std::hypot(step.x + step.dx - x, step.y + step.dy - y), .0005);
  const auto result =
    distance_controller::correction_feedback(step, x, y, yaw, yaw - step.yaw, .008726646, 3, .008);
  ASSERT_FALSE(result.fault);
  EXPECT_FALSE(result.arrived);
  EXPECT_TRUE(result.remeasure);
  EXPECT_DOUBLE_EQ(result.vx, 0);
  EXPECT_DOUBLE_EQ(result.vy, 0);
  EXPECT_DOUBLE_EQ(result.wz, 0);
}

TEST(StartAdjustment, StepHeadingUsesFinalHalfDegreeLimitAndRespectsTighterConfiguration)
{
  const distance_controller::StartCorrection step{-.131842, .361371, .207671,    -.001031,
                                                  .004893,  "odom",  "base_link"};
  const auto field = distance_controller::correction_feedback(
    step, -.132332, .362033, .202503, .202503 - .208094, .008726646, 2.3, .008);
  EXPECT_FALSE(field.fault);
  EXPECT_FALSE(field.arrived);
  EXPECT_FALSE(field.remeasure);
  EXPECT_GT(field.wz, 0);
  EXPECT_LE(std::hypot(field.vx, field.vy), .004000001);
  for (double drift : {-.0088, .0088}) {
    const auto outside = distance_controller::correction_feedback(
      step, step.x, step.y, step.yaw + drift, 0, .008726646, 1, .008);
    EXPECT_STREQ(outside.fault, "START_CORRECTION_HEADING");
    EXPECT_DOUBLE_EQ(outside.wz, 0);
  }
  const auto tighter = distance_controller::correction_feedback(
    step, step.x, step.y, step.yaw + .0041, 0, .004, 1, .008);
  EXPECT_STREQ(tighter.fault, "START_CORRECTION_HEADING");
  const auto global = distance_controller::correction_feedback(
    step, step.x, step.y, step.yaw, .009, .008726646, 1, .008);
  EXPECT_STREQ(global.fault, "START_CORRECTION_HEADING");
}

TEST(StartVerification, TightJointMajoritySurvivesUpToThreeModerateSpikes)
{
  std::vector<StartSample> samples(15, {.26, .28, 0});
  samples[0] = {.272, .28, 0};
  samples[1] = {.248, .28, 0};
  samples[2] = {.26, .293, 0};
  const auto summary = summarize_start(samples);
  EXPECT_TRUE(summary.stable);
  EXPECT_EQ(summary.retained, 15u);
  EXPECT_EQ(summary.consensus, 12u);
  EXPECT_GT(summary.spread[0], .01);
  EXPECT_GT(summary.spread[1], .01);
  EXPECT_DOUBLE_EQ(summary.consensus_spread[0], 0.0);
  EXPECT_DOUBLE_EQ(summary.median[0], .26);
  EXPECT_DOUBLE_EQ(summary.median[1], .28);
}

TEST(StartVerification, FourConflictingFramesCannotBeDiscarded)
{
  std::vector<StartSample> samples(15, {.26, .28, 0});
  for (unsigned i = 0; i < 4; ++i) samples[i][1] += .012;
  const auto summary = summarize_start(samples);
  EXPECT_FALSE(summary.stable);
  EXPECT_EQ(summary.consensus, 0u);
  EXPECT_DOUBLE_EQ(summary.consensus_spread[0], 0.0);
}

TEST(StartVerification, SeparateAxisMajoritiesAreNotJointEvidence)
{
  std::vector<StartSample> samples(15, {.26, .28, 0});
  for (unsigned i = 0; i < 3; ++i) samples[i][0] += .015;
  for (unsigned i = 3; i < 6; ++i) samples[i][1] += .015;
  for (unsigned i = 6; i < 9; ++i) samples[i][2] += .034;
  EXPECT_FALSE(summarize_start(samples).stable);
}

TEST(StartVerification, ConsensusNeverChangesFullBatchMedianOrTargetTolerance)
{
  std::vector<StartSample> samples(15, {.268, .289, .015});
  samples[0][0] += .012;
  samples[1][1] += .012;
  samples[2][0] -= .012;
  const auto summary = summarize_start(samples);
  ASSERT_TRUE(summary.stable);
  EXPECT_DOUBLE_EQ(summary.median[0], .268);
  EXPECT_DOUBLE_EQ(summary.median[1], .289);
  EXPECT_DOUBLE_EQ(summary.median[2], .015);
  EXPECT_GT(std::abs(summary.median[0] - .26), .005);
  EXPECT_GT(std::abs(summary.median[1] - .28), .005);
  EXPECT_GT(std::abs(summary.median[2]), .008726646);
  EXPECT_TRUE(distance_controller::start_adjustment(summary, .26, .28, .005).has_value());
}

TEST(StartVerification, CoarseDiscardedFramesStillConsumeTheThreeFrameAllowance)
{
  std::vector<StartSample> samples(15, {.26, .28, 0});
  samples[0][0] += .05;
  samples[1][0] += .05;
  samples[2][1] += .012;
  EXPECT_TRUE(summarize_start(samples).stable);
  samples[3][1] += .012;
  EXPECT_FALSE(summarize_start(samples).stable);
}

TEST(StartVerification, JointConsensusAcceptanceIsIndependentOfSampleOrder)
{
  std::vector<StartSample> samples(15, {.26, .28, 0});
  samples[0][0] += .012;
  samples[1][1] += .013;
  samples[2][0] -= .012;
  const auto baseline = summarize_start(samples);
  ASSERT_TRUE(baseline.stable);
  for (unsigned i = 0; i < samples.size(); ++i) {
    std::rotate(samples.begin(), samples.begin() + 1, samples.end());
    const auto summary = summarize_start(samples);
    EXPECT_TRUE(summary.stable);
    EXPECT_EQ(summary.median, baseline.median);
    EXPECT_EQ(summary.spread, baseline.spread);
  }
}

TEST(StartVerification, ConsensusDoesNotRelaxDistanceOrAngleSpreadThresholds)
{
  for (unsigned axis = 0; axis < 3; ++axis) {
    const double limit = axis == 2 ? .034906585 : .01;
    std::vector<StartSample> samples(15, {0, 0, 0});
    for (unsigned i = 0; i < 3; ++i) samples[i][axis] = 1.;
    samples[3][axis] = limit;
    EXPECT_TRUE(summarize_start(samples).stable);
    samples[3][axis] += .000001;
    EXPECT_FALSE(summarize_start(samples).stable);
  }
}

TEST(StartVerification, NonfiniteAndModerateOutliersShareTheSameEvidenceBudget)
{
  std::vector<StartSample> samples(15, {.26, .28, 0});
  samples[0][0] = std::numeric_limits<double>::quiet_NaN();
  samples[1][0] = std::numeric_limits<double>::infinity();
  samples[2][1] += .012;
  EXPECT_TRUE(summarize_start(samples).stable);
  samples[3][1] += .012;
  EXPECT_FALSE(summarize_start(samples).stable);
}

TEST(StartAdjustment, BiasedStepEntryRestoresGlobalHeadingWithoutRelaxingEitherGuard)
{
  const double reference = -.0135141;
  const distance_controller::StartCorrection step{-.003051, -.000023, -.019397,   -.004999,
                                                  .000097,  "odom",   "base_link"};
  const auto initial = distance_controller::correction_feedback(
    step, step.x, step.y, step.yaw, step.yaw - reference, .008726646, 0, .004);
  ASSERT_FALSE(initial.fault);
  EXPECT_NEAR(initial.wz, .01058922, 1e-10);
  EXPECT_LE(std::hypot(initial.vx, initial.vy), .004000001);
  // Current yaw remains below the global limit: steer toward its reference even
  // when that direction is opposite to restoring the step-entry orientation.
  const double yaw = -.018;
  const auto inward = distance_controller::correction_feedback(
    step, step.x, step.y, yaw, yaw - reference, .008726646, .5, .004);
  ASSERT_FALSE(inward.fault);
  EXPECT_GT(inward.wz, 0);
  const auto field_fault = distance_controller::correction_feedback(
    step, -.00423244, .000291882, -.0235421, -.0235421 - reference, .008726646, .75, .004);
  ASSERT_STREQ(field_fault.fault, "START_CORRECTION_HEADING");
  EXPECT_DOUBLE_EQ(field_fault.wz, 0);
  EXPECT_DOUBLE_EQ(field_fault.vx, 0);
  EXPECT_DOUBLE_EQ(field_fault.vy, 0);
}
