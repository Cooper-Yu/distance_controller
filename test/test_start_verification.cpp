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
  samples.back()[0] += .010408;
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
      EXPECT_NEAR((*delta)[0], -.00892674018, 1e-12);
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
    EXPECT_NEAR(std::hypot((*delta)[0], (*delta)[1]), .01, 1e-12);
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
  EXPECT_NEAR(initial.vx, -.008, 1e-12);
  EXPECT_NEAR(initial.vy, 0, 1e-12);
  const auto done =
    distance_controller::correction_feedback(step, 1, 1.9912, half_pi, 0, .008726646, 2, .008);
  EXPECT_TRUE(done.arrived);
  EXPECT_DOUBLE_EQ(done.vx, 0);
  EXPECT_DOUBLE_EQ(done.vy, 0);
}

TEST(StartAdjustment, FaultBoundsAlwaysReturnZero)
{
  distance_controller::StartCorrection step{0, 0, 0, -.009, 0, "odom", "base_link"};
  const std::array<std::array<double, 5>, 6> cases{
    {{0, 0, 0, 0, 4.01},
     {-.013, 0, 0, 0, 1},
     {0, .0031, 0, 0, 1},
     {0, 0, .0051, 0, 1},
     {0, 0, 0, .009, 1},
     {std::numeric_limits<double>::quiet_NaN(), 0, 0, 0, 1}}};
  for (const auto & value : cases) {
    const auto result = distance_controller::correction_feedback(
      step, value[0], value[1], value[2], value[3], .008726646, value[4], .008);
    EXPECT_TRUE(result.fault);
    EXPECT_FALSE(result.arrived);
    EXPECT_DOUBLE_EQ(result.vx, 0);
    EXPECT_DOUBLE_EQ(result.vy, 0);
  }
}

TEST(StartAdjustment, SmallOvershootReversesWithinTheSameFrozenBoundedTarget)
{
  const distance_controller::StartCorrection step{0, 0, 0, -.009, 0, "odom", "base_link"};
  const auto result =
    distance_controller::correction_feedback(step, -.0097, 0, 0, 0, .008726646, 2, .008);
  EXPECT_FALSE(result.fault);
  EXPECT_FALSE(result.arrived);
  EXPECT_NEAR(result.vx, .0014, 1e-12);
  EXPECT_DOUBLE_EQ(result.vy, 0.0);
  EXPECT_DOUBLE_EQ(step.dx, -.009);
}
