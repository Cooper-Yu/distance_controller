/** @file
 * @brief Verify ancestry, repeated visits and return-policy conversion without ROS.
 */
#include <gtest/gtest.h>

#include "distance_controller/route_history.hpp"
using namespace distance_controller;

TEST(History, CompletedChainReturnsThroughRecordedStarts)
{
  RouteHistory h;
  h.initialize({0, 0, 0.2});
  auto ab = make_ab_segment(1, 0.1, 1);
  auto bc = make_bc_segment(0.5, 0.08, 2);
  h.complete(ab, {0, 0, 0.2}, {0.98, 0.20, 0.2}, 0.2);
  h.complete(bc, {0.98, 0.20, 0.2}, {1.08, -0.29, 0.2}, 0.2);
  auto plan = h.reverse_plan(h.count_to("A", false, 0));
  ASSERT_EQ(plan.size(), 2u);
  EXPECT_EQ(plan[0].to, "B");
  EXPECT_DOUBLE_EQ(plan[0].motion.dx, 0.98);
  EXPECT_DOUBLE_EQ(plan[0].motion.dy, 0.20);
  EXPECT_DOUBLE_EQ(plan[0].motion.max_speed, 0.08);
  EXPECT_EQ(plan[1].to, "A");
  EXPECT_DOUBLE_EQ(plan[1].motion.dx, 0);
  h.complete(plan[0], {1.08, -0.29, 0.2}, {0.985, 0.20, 0.2}, 0.2);
  EXPECT_EQ(h.count_to("A", false, 0), 1u);
  h.complete(plan[1], {0.985, 0.20, 0.2}, {0.005, 0, 0.2}, 0.2);
  EXPECT_THROW(h.reverse_plan(1), std::invalid_argument);
  EXPECT_NE(h.describe().find("completed=4 outward=0"), std::string::npos);
}

TEST(History, RepeatedNamesRequireVisitIdAndBranchesRemainAuditable)
{
  RouteHistory h;
  h.initialize({});
  auto ab = make_ab_segment(1, .1, 0), ba = make_ba_segment(1, .1, 0);
  h.complete(ab, {}, {1, 0, 0}, 0);
  h.complete(ba, {1, 0, 0}, {}, 0);
  h.complete(ab, {}, {1, 0, 0}, 0);
  EXPECT_THROW(h.count_to("B", false, 0), std::invalid_argument);
  EXPECT_EQ(h.count_to("", true, 1), 2u);
  auto plan = h.reverse_plan(2);
  for (const auto & step : plan) h.complete(step, {}, {}, 0);
  auto bd = ab;
  bd.from = "B";
  bd.to = "D";
  h.complete(bd, {1, 0, 0}, {2, 0, 0}, 0);
  EXPECT_EQ(h.count_to("A", false, 0), 2u);
  EXPECT_THROW(h.count_to("", true, 3), std::invalid_argument);
  EXPECT_NE(h.describe().find("edge=3"), std::string::npos);
}

TEST(History, LaserArrivalBecomesPositionAndLimitsArePreserved)
{
  RouteHistory h;
  h.initialize({});
  auto step = make_ab_segment(1, .03, .5);
  step.completion = CompletionKind::FrontWall;
  step.side_centering = true;
  step.timeout = 31;
  step.max_travel = .7;
  h.complete(step, {.01, .02, 0}, {.4, .03, 0}, 0);
  auto reverse = h.reverse_plan(1).front();
  EXPECT_EQ(reverse.completion, CompletionKind::Position);
  EXPECT_EQ(reverse.frame, MotionFrame::Odom);
  EXPECT_FALSE(reverse.side_centering);
  EXPECT_DOUBLE_EQ(reverse.timeout, 31);
  EXPECT_DOUBLE_EQ(reverse.max_travel, .7);
  EXPECT_DOUBLE_EQ(reverse.motion.dx, .01);
}

TEST(History, InvalidReturnNeverConsumesThePath)
{
  RouteHistory h;
  h.initialize({});
  auto ab = make_ab_segment(1, .1, 0);
  h.complete(ab, {}, {1, 0, 0}, 0);
  EXPECT_THROW(h.reverse_plan(0), std::invalid_argument);
  EXPECT_THROW(h.reverse_plan(2), std::invalid_argument);
  EXPECT_THROW(h.count_to("D", false, 0), std::invalid_argument);
  auto reverse = h.reverse_plan(1).front();
  reverse.reverse_of = 99;
  EXPECT_THROW(h.complete(reverse, {}, {}, 0), std::logic_error);
  EXPECT_EQ(h.reverse_plan(1).front().reverse_of, 1u);
}
