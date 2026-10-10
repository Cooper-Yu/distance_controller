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
