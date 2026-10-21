/** @file @brief Passive per-attempt accounting for precise-start evidence loss. */
#pragma once
#include <cstddef>

#include "distance_controller/start_verification.hpp"

namespace distance_controller
{
/// Observation only: no thresholds, clocks, acceptance decisions or control outputs.
struct StartDiagnostics
{
  static constexpr unsigned LinearSpeed = 1, AngularSpeed = 2, AnchorMotion = 4;
  unsigned stop_mask{0}, last_stop_mask{0};
  unsigned reset_events{0}, linear_resets{0}, angular_resets{0}, anchor_resets{0};
  unsigned blocked_ticks{0}, hold_ticks{0}, no_new_scan_ticks{0};
  unsigned accepted{0}, batches{0}, scan_resets{0}, odom_resets{0};
  std::size_t discarded{0};
  StartSummary last_batch{};

  /// Count a new blocking episode/reason change, rather than every blocked timer tick.
  void observe_stop(unsigned mask, std::size_t sample_count)
  {
    if (mask) {
      ++blocked_ticks;
      if (mask != stop_mask || sample_count) {
        ++reset_events;
        if (mask & LinearSpeed) ++linear_resets;
        if (mask & AngularSpeed) ++angular_resets;
        if (mask & AnchorMotion) ++anchor_resets;
      }
      last_stop_mask = mask;
      discarded += sample_count;
    }
    stop_mask = mask;
  }

  /// Remember the actual computed summary; no re-filtering or acceptance change.
  void observe_batch(const StartSummary & summary)
  {
    ++batches;
    last_batch = summary;
  }
};
}  // namespace distance_controller
