/** @file @brief Bounded odometry feedback for a stable-median precise-start translation. */
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <string>

#include "distance_controller/start_verification.hpp"

namespace distance_controller
{
/// Plan in body axes: rear error is x, right error is y. Keep accepted axes still.
inline std::optional<std::array<double, 2>> start_adjustment(
  const StartSummary & summary, double right_target, double rear_target, double tolerance)
{
  if (!summary.stable) return std::nullopt;
  std::array<double, 2> delta{rear_target - summary.median[1], right_target - summary.median[0]};
  for (double & value : delta) {
    if (!std::isfinite(value) || std::abs(value) > .02) return std::nullopt;
    if (std::abs(value) <= tolerance) value = 0.0;
  }
  const double length = std::hypot(delta[0], delta[1]);
  if (length > .01)
    for (double & value : delta) value *= .01 / length;
  return delta;
}

/// Frozen odom anchor and displacement; no laser sample can change an active target.
struct StartCorrection
{
  double x{}, y{}, yaw{}, dx{}, dy{};
  std::string frame, child_frame;
};

/// A fault or arrival always returns zero velocity; velocity is in current body axes.
struct StartCorrectionFeedback
{
  const char * fault{nullptr};
  bool arrived{false};
  double vx{}, vy{};
};

/// Evaluate one fresh-feedback tick without changing clocks, budgets or the frozen step.
inline StartCorrectionFeedback correction_feedback(
  const StartCorrection & step, double x, double y, double yaw, double heading_error,
  double heading_tolerance, double elapsed, double cap)
{
  const std::array<double, 12> inputs{step.x,  step.y, step.yaw, step.dx,       step.dy,
                                      x,       y,      yaw,      heading_error, heading_tolerance,
                                      elapsed, cap};
  if (!std::all_of(inputs.begin(), inputs.end(), [](double v) { return std::isfinite(v); }))
    return {"START_CORRECTION_NONFINITE"};
  const double moved_x = x - step.x, moved_y = y - step.y;
  const double length = std::hypot(step.dx, step.dy);
  if (length <= 0 || length > .010000000001 || cap <= 0 || heading_tolerance <= 0)
    return {"START_CORRECTION_INVALID_PLAN"};
  if (elapsed < 0 || elapsed > 4.0) return {"START_CORRECTION_TIMEOUT"};
  if (std::hypot(moved_x, moved_y) > .012) return {"START_CORRECTION_TRAVEL_LIMIT"};
  if (
    std::abs(std::remainder(yaw - step.yaw, 2 * 3.141592653589793)) > .005 ||
    std::abs(heading_error) > heading_tolerance)
    return {"START_CORRECTION_HEADING"};
  if (std::abs(moved_x * step.dy - moved_y * step.dx) / length > .003)
    return {"START_CORRECTION_CROSS_TRACK"};
  const double ex = step.dx - moved_x, ey = step.dy - moved_y;
  if (std::hypot(ex, ey) <= .0005) return {nullptr, true};
  const double scale = std::min(2.0, std::min(cap, .008) / std::hypot(ex, ey));
  return {
    nullptr, false, scale * (std::cos(yaw) * ex + std::sin(yaw) * ey),
    scale * (-std::sin(yaw) * ex + std::cos(yaw) * ey)};
}
}  // namespace distance_controller
