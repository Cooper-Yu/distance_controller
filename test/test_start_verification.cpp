/** @file @brief Acceptance and rejection of stopped scan batches. */
#include <gtest/gtest.h>

#include <limits>

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
