/** @file
 * @brief Bounded dominant-line selection for heading estimation; never filters obstacle scans.
 */
#pragma once
#include <cmath>
#include <utility>
#include <vector>

namespace distance_controller
{
/// Select a 12 mm consensus supported by >=80% of finite window returns.
/// At most 33 candidate points bound cost; retain all points when testing each line.
/// Empty means no dominant wall. Caller still checks coverage, TLS RMS, span and angle.
/// @param points Finite base-frame window returns in meters; never mutated.
/// @param minimum_baseline Minimum candidate pair separation in meters.
/// @return Consensus points, or an empty vector when no dominant wall is found.
inline std::vector<std::pair<double, double>> dominant_wall(
  const std::vector<std::pair<double, double>> & points, double minimum_baseline)
{
  std::vector<std::pair<double, double>> best;
  if (points.size() < 8) return best;
  const std::size_t stride = (points.size() + 31) / 32;
  for (std::size_t i = 0; i < points.size(); i += stride) {
    for (std::size_t j = i + stride; j < points.size(); j += stride) {
      const double dx = points[j].first - points[i].first;
      const double dy = points[j].second - points[i].second;
      const double length = std::hypot(dx, dy);
      if (length < minimum_baseline) continue;
      std::vector<std::pair<double, double>> inliers;
      for (const auto & p : points) {
        const double residual =
          std::abs(-dy * (p.first - points[i].first) + dx * (p.second - points[i].second)) / length;
        if (residual <= 0.012) inliers.push_back(p);
      }
      if (inliers.size() > best.size()) best = std::move(inliers);
    }
  }
  if (best.size() * 5 < points.size() * 4) best.clear();
  return best;
}
}  // namespace distance_controller
