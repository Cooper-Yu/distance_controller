/** @file
 * @brief Robust stopped right/rear distance and wall-angle batch statistics. */
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace distance_controller
{
/// One scan: base-origin right/rear distances (m), body wall angle (rad).
using StartSample = std::array<double, 3>;

/// At least 12 of exactly 15 scans must agree before a batch is usable.
struct StartSummary
{
  StartSample median{};
  StartSample spread{};
  unsigned retained{0};
  bool stable{false};
};

/// Median of a nonempty copied scalar array; input remains unchanged.
inline double start_median(std::vector<double> values)
{
  std::sort(values.begin(), values.end());
  const auto n = values.size();
  return (values[n / 2] + values[(n - 1) / 2]) * 0.5;
}

/// Reject joint outliers (2 cm / 2 deg), then require 1 cm / 2 deg spread.
/// Nonfinite fit samples count against the required 12/15 consensus.
inline StartSummary summarize_start(const std::vector<StartSample> & samples)
{
  StartSummary result;
  if (samples.size() != 15) return result;
  std::array<std::vector<double>, 3> axes;
  for (const auto & p : samples) {
    if (!std::all_of(p.begin(), p.end(), [](double v) { return std::isfinite(v); })) continue;
    for (unsigned i = 0; i < 3; ++i) axes[i].push_back(p[i]);
  }
  if (axes[0].size() < 12) return result;
  StartSample anchor;
  for (unsigned i = 0; i < 3; ++i) anchor[i] = start_median(axes[i]);
  for (auto & axis : axes) axis.clear();
  for (const auto & p : samples) {
    if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2])) continue;
    if (
      std::abs(p[0] - anchor[0]) > .02 || std::abs(p[1] - anchor[1]) > .02 ||
      std::abs(p[2] - anchor[2]) > .034906585)
      continue;
    for (unsigned i = 0; i < 3; ++i) axes[i].push_back(p[i]);
  }
  result.retained = axes[0].size();
  if (result.retained < 12) return result;
  for (unsigned i = 0; i < 3; ++i) {
    result.median[i] = start_median(axes[i]);
    const auto limits = std::minmax_element(axes[i].begin(), axes[i].end());
    result.spread[i] = *limits.second - *limits.first;
  }
  result.stable =
    result.spread[0] <= .01 && result.spread[1] <= .01 && result.spread[2] <= .034906585;
  return result;
}
}  // namespace distance_controller
