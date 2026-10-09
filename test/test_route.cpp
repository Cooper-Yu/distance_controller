/** @file
 * @brief Check motion direction, validation and named-route continuity without ROS execution.
 */
#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include "distance_controller/route.hpp"
using distance_controller::compose_route;
using distance_controller::PlanarMotion;

TEST(Motion, DirectionAndParameters)
{
  const auto forward = PlanarMotion::move_forward(0.93, 0.1, 1.0);
  const auto backward = PlanarMotion::move_backward(0.93, 0.1, 1.0);
  const auto left = PlanarMotion::move_left(0.52, 0.08, 0.5);
  const auto right = PlanarMotion::move_right(0.52, 0.08, 0.5);
  EXPECT_DOUBLE_EQ(forward.dx, 0.93);
  EXPECT_DOUBLE_EQ(forward.dy, 0.0);
  EXPECT_DOUBLE_EQ(backward.dx, -forward.dx);
  EXPECT_DOUBLE_EQ(left.dy, 0.52);
  EXPECT_DOUBLE_EQ(right.dy, -left.dy);
  EXPECT_DOUBLE_EQ(left.dx, 0.0);
  EXPECT_DOUBLE_EQ(right.max_speed, 0.08);
  EXPECT_DOUBLE_EQ(right.dwell, 0.5);
  const auto diagonal = PlanarMotion::move_relative(-0.3, 0.4, 0.2, 0.0);
  EXPECT_DOUBLE_EQ(diagonal.dx, -0.3);
  EXPECT_DOUBLE_EQ(diagonal.dy, 0.4);
}

TEST(Motion, InvalidInputs)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const double distance : {0.0, -1.0, nan, inf}) {
    EXPECT_THROW(PlanarMotion::move_forward(distance, 0.1, 1.0), std::invalid_argument);
    EXPECT_THROW(PlanarMotion::move_backward(distance, 0.1, 1.0), std::invalid_argument);
    EXPECT_THROW(PlanarMotion::move_left(distance, 0.1, 1.0), std::invalid_argument);
    EXPECT_THROW(PlanarMotion::move_right(distance, 0.1, 1.0), std::invalid_argument);
  }
  EXPECT_THROW(PlanarMotion::move_relative(0, 0, 0.1, 1), std::invalid_argument);
  EXPECT_THROW(PlanarMotion::move_relative(nan, 1, 0.1, 1), std::invalid_argument);
  EXPECT_THROW(PlanarMotion::move_relative(1, inf, 0.1, 1), std::invalid_argument);
  for (const double speed : {0.0, -1.0, nan, inf}) {
    EXPECT_THROW(PlanarMotion::move_forward(1, speed, 1), std::invalid_argument);
  }
  for (const double dwell : {-1.0, nan, inf}) {
    EXPECT_THROW(PlanarMotion::move_forward(1, 0.1, dwell), std::invalid_argument);
  }
}

TEST(Route, CompositionsAndEndpoints)
{
  const auto ab = compose_route({"AB"}, 0.93, 0.52, 0.1, 1);
  ASSERT_EQ(ab.size(), 1u);
  EXPECT_EQ(ab.front().from, "A");
  EXPECT_EQ(ab.back().to, "B");
  const auto back = compose_route({"AB", "BA"}, 0.93, 0.52, 0.1, 1);
  EXPECT_DOUBLE_EQ(back[0].motion.dx + back[1].motion.dx, 0);
  const auto corner = compose_route({"AB", "BC"}, 0.93, 0.52, 0.1, 1);
  EXPECT_EQ(corner.back().to, "C");
  EXPECT_DOUBLE_EQ(corner.back().motion.dy, -0.52);
  const auto full = compose_route({"AB", "BC", "CB", "BA"}, 0.93, 0.52, 0.1, 1);
  double x = 0, y = 0;
  for (const auto & segment : full) {
    x += segment.motion.dx;
    y += segment.motion.dy;
    EXPECT_DOUBLE_EQ(segment.motion.max_speed, 0.1);
    EXPECT_DOUBLE_EQ(segment.motion.dwell, 1);
  }
  EXPECT_DOUBLE_EQ(x, 0);
  EXPECT_DOUBLE_EQ(y, 0);
  const auto repeated = compose_route({"AB", "BA", "AB"}, 0.93, 0.52, 0.1, 1);
  EXPECT_EQ(repeated.size(), 3u);
  EXPECT_DOUBLE_EQ(repeated[0].motion.dx, repeated[2].motion.dx);
}

TEST(Route, RejectDisconnectedOrUnknown)
{
  EXPECT_THROW(compose_route({}, 1, 1, 0.1, 1), std::invalid_argument);
  EXPECT_THROW(compose_route({"BC"}, 1, 1, 0.1, 1), std::invalid_argument);
  EXPECT_THROW(compose_route({"AB", "CB"}, 1, 1, 0.1, 1), std::invalid_argument);
  EXPECT_THROW(compose_route({"AB", "AB"}, 1, 1, 0.1, 1), std::invalid_argument);
  EXPECT_THROW(compose_route({"AD"}, 1, 1, 0.1, 1), std::invalid_argument);
}

TEST(StepPolicy, IndependentEdgesAndFeedback)
{
  using namespace distance_controller;
  auto ab = make_ab_segment(0.9, 0.1, 1);
  auto bc = make_bc_segment(0.5, 0.08, 0.5);
  ab.side_centering = true;
  EXPECT_NO_THROW(validate_segment(ab));
  EXPECT_FALSE(bc.side_centering);
  bc.side_centering = true;
  EXPECT_THROW(validate_segment(bc), std::invalid_argument);
  ab.completion = CompletionKind::FrontWall;
  EXPECT_NO_THROW(validate_segment(ab));
  auto ba = make_ba_segment(0.9, 0.1, 1);
  ba.completion = CompletionKind::FrontWall;
  EXPECT_THROW(validate_segment(ba), std::invalid_argument);
}

TEST(StepPolicy, AbsoluteOriginAndBounds)
{
  using namespace distance_controller;
  RouteSegment point{"B", "D", {0, 0, 0.1, 0}};
  point.frame = MotionFrame::Odom;
  EXPECT_NO_THROW(validate_segment(point));
  point.frame = MotionFrame::Heading;
  EXPECT_THROW(validate_segment(point), std::invalid_argument);
  point = make_ab_segment(0.2, 0.1, 0);
  for (double invalid : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN()}) {
    point.timeout = invalid;
    EXPECT_THROW(validate_segment(point), std::invalid_argument);
    point.timeout = 10;
    point.max_travel = invalid;
    EXPECT_THROW(validate_segment(point), std::invalid_argument);
    point.max_travel = 1;
  }
}
