/** @file
 * @brief Stopped multi-scan wall direction in odom coordinates. */
#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

namespace distance_controller
{
/// A bounded recent window; angles are unwrapped around the first sample.
struct WallHeadingWindow
{
  std::vector<std::pair<double, double>> samples;  ///< Receipt seconds and world angle.

  void clear()
  {
    samples.clear();
  }  ///< Discard pre-motion/pre-verification evidence.

  /// @brief Add one accepted distinct scan; input angle is in odom radians.
  /// @param now Steady receipt seconds.
  /// @param angle Odom yaw plus fitted body wall angle.
  void add(double now, double angle)
  {
    samples.erase(
      std::remove_if(
        samples.begin(), samples.end(), [now](const auto & p) { return now - p.first > 1.5; }),
      samples.end());
    if (!samples.empty()) {
      const double anchor = samples.front().second;
      angle = anchor + std::atan2(std::sin(angle - anchor), std::cos(angle - anchor));
    }
    samples.emplace_back(now, angle);
    if (samples.size() > 16) samples.erase(samples.begin());
  }

  /// @brief Require eight recent scans spanning 0.5 s and 80% within 0.02 rad of median.
  /// @param now Steady receipt seconds.
  /// @param angle Receives median world direction on success.
  /// @return True only when recent multi-frame evidence is concentrated.
  bool estimate(double now, double & angle) const
  {
    if (
      samples.size() < 8 || now - samples.back().first > .25 || now - samples.front().first > 1.5 ||
      samples.back().first - samples.front().first < .5)
      return false;
    std::vector<double> values;
    for (const auto & p : samples) values.push_back(p.second);
    std::sort(values.begin(), values.end());
    angle = (values[(values.size() - 1) / 2] + values[values.size() / 2]) / 2;
    std::size_t count = 0;
    for (double value : values)
      if (std::abs(value - angle) <= .02) ++count;
    return count * 5 >= values.size() * 4;
  }
};
}  // namespace distance_controller
