#include <gtest/gtest.h>

#include "distance_controller/robust_wall.hpp"

TEST(RobustWall, RejectMinorityAndKeepInput)
{
  std::vector<std::pair<double, double>> points;
  for (int i = 0; i < 100; ++i) points.emplace_back(i * .004, .3 + i * .0003);
  for (int i = 0; i < 5; ++i) points.emplace_back(.2 + i * .004, .48);
  const auto selected = distance_controller::dominant_wall(points, .18);
  EXPECT_EQ(selected.size(), 100u);
  EXPECT_EQ(points.size(), 105u);
}

TEST(RobustWall, RejectCompetingSurfaces)
{
  std::vector<std::pair<double, double>> points;
  for (int i = 0; i < 40; ++i) points.emplace_back(i * .01, .3);
  for (int i = 0; i < 40; ++i) points.emplace_back(i * .01, .5);
  EXPECT_TRUE(distance_controller::dominant_wall(points, .18).empty());
}

TEST(RobustWall, RejectShortCluster)
{
  std::vector<std::pair<double, double>> points;
  for (int i = 0; i < 40; ++i) points.emplace_back(i * .001, .3);
  EXPECT_TRUE(distance_controller::dominant_wall(points, .18).empty());
}

#include "distance_controller/wall_heading_window.hpp"

TEST(WallWindow, NoiseAcrossPiAndExpiry)
{
  distance_controller::WallHeadingWindow window;
  for (int i = 0; i < 12; ++i) {
    const double a = 3.14 + (i % 2 ? .013 : -.013);
    window.add(i * .08, std::atan2(std::sin(a), std::cos(a)));
  }
  double angle = 0;
  ASSERT_TRUE(window.estimate(.9, angle));
  EXPECT_NEAR(angle, 3.14, .001);
  EXPECT_FALSE(window.estimate(1.3, angle));
  window.clear();
  EXPECT_FALSE(window.estimate(1.3, angle));
}

TEST(WallWindow, RejectUnstableAndTooFew)
{
  distance_controller::WallHeadingWindow window;
  double angle = 0;
  for (int i = 0; i < 7; ++i) window.add(i * .1, .1);
  EXPECT_FALSE(window.estimate(.65, angle));
  window.clear();
  for (int i = 0; i < 12; ++i) window.add(i * .08, i % 2 ? .1 : -.1);
  EXPECT_FALSE(window.estimate(.9, angle));
}