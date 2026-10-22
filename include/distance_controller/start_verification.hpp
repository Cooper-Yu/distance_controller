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

/// Keep exactly the latest fifteen accepted fresh scans; time/stop gates belong to the caller.
inline void append_start_sample(std::vector<StartSample> & samples, const StartSample & sample)
{
  samples.push_back(sample);
  if (samples.size() > 15) samples.erase(samples.begin());
}

/// At least 12 of exactly 15 scans must agree before a batch is usable.
struct StartSummary
{
  StartSample median{};
  StartSample spread{};
  unsigned retained{0};
  unsigned consensus{0};
  StartSample consensus_spread{};
  bool stable{false};
};

/// Median of a nonempty copied scalar array; input remains unchanged.
inline double start_median(std::vector<double> values)
{
  std::sort(values.begin(), values.end());
  const auto n = values.size();
  return (values[n / 2] + values[(n - 1) / 2]) * 0.5;
}

/// Apply the same distance/angle spread limits to every member of a joint subset.
inline bool start_spread_is_stable(const StartSample & spread)
{
  return spread[0] <= .01 && spread[1] <= .01 && spread[2] <= .034906585;
}

/// Find twelve jointly agreeing scans, never separate per-axis subsets.
/// At most C(15,12)=455 candidates; full-batch medians are never replaced.
inline bool find_start_consensus(const std::vector<StartSample> & samples, StartSample & spread)
{
  if (samples.size() < 12 || samples.size() > 15) return false;
  std::array<unsigned, 12> selected{};
  for (unsigned i = 0; i < selected.size(); ++i) selected[i] = i;
  while (true) {
    StartSample low = samples[selected[0]], high = low;
    for (const auto index : selected) {
      for (unsigned axis = 0; axis < 3; ++axis) {
        low[axis] = std::min(low[axis], samples[index][axis]);
        high[axis] = std::max(high[axis], samples[index][axis]);
      }
    }
    StartSample candidate{};
    for (unsigned axis = 0; axis < 3; ++axis) candidate[axis] = high[axis] - low[axis];
    if (start_spread_is_stable(candidate)) {
      spread = candidate;
      return true;
    }
    int cursor = 11;
    while (cursor >= 0 && selected[cursor] == samples.size() - 12 + cursor) --cursor;
    if (cursor < 0) return false;
    ++selected[cursor];
    for (unsigned i = cursor + 1; i < selected.size(); ++i) selected[i] = selected[i - 1] + 1;
  }
}

/// Reject coarse joint outliers (2 cm / 2 deg); require >=12/15 in one tight joint group.
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
  std::vector<StartSample> retained;
  for (const auto & p : samples) {
    if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2])) continue;
    if (
      std::abs(p[0] - anchor[0]) > .02 || std::abs(p[1] - anchor[1]) > .02 ||
      std::abs(p[2] - anchor[2]) > .034906585)
      continue;
    retained.push_back(p);
    for (unsigned i = 0; i < 3; ++i) axes[i].push_back(p[i]);
  }
  result.retained = axes[0].size();
  if (result.retained < 12) return result;
  for (unsigned i = 0; i < 3; ++i) {
    result.median[i] = start_median(axes[i]);
    const auto limits = std::minmax_element(axes[i].begin(), axes[i].end());
    result.spread[i] = *limits.second - *limits.first;
  }
  result.stable = start_spread_is_stable(result.spread);
  if (result.stable) {
    result.consensus = result.retained;
    result.consensus_spread = result.spread;
  } else if (find_start_consensus(retained, result.consensus_spread)) {
    result.consensus = 12;
    result.stable = true;
  }
  return result;
}
}  // namespace distance_controller
