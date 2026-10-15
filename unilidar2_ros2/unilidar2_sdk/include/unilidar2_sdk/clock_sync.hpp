#pragma once

#include <chrono>
#include <cstdint>

#include "messages.hpp"

namespace unilidar2
{
/// @brief Maps L2 timestamps onto the host's system clock.
///
/// The L2's timestamp clock can't be trusted to run at real time (observed running at 0.5x), and even a correct clock
/// drifts away from the host after a one time sync. This continuously fits host receive time against device timestamp
/// with an exponentially weighted linear regression, which estimates both the rate and the offset between the clocks.
/// The mapped timestamps keep the L2's packet to packet timing while staying locked to host time.
///
/// Mapped timestamps include the average transport latency, so they're slightly later than the true sample time.
class ClockSync
{
private:
  // Per sample weight decay. Roughly 10 seconds of history at the L2's ~470 packets per second.
  static constexpr double DECAY = 0.9998;
  // Minimum spread of device time (standard deviation, seconds) before the fitted rate is trusted.
  static constexpr double MIN_SPREAD = 0.1;
  // A prediction error larger than this means the device clock jumped (time sync, reboot), so start over.
  static constexpr double RESET_ERROR = 0.5;

  bool initialized_ = false;

  // Reference points; the fit works in seconds relative to these to keep precision.
  TimeStamp device_ref_{};
  int64_t host_ref_ns_ = 0;

  // Exponentially weighted sample count, means, and co-moments (x = device, y = host).
  double weight_ = 0;
  double mean_x_ = 0;
  double mean_y_ = 0;
  double cov_xx_ = 0;
  double cov_xy_ = 0;

  void reset(const TimeStamp & device, int64_t host_ns);
  double device_seconds(const TimeStamp & device) const;
  bool fit_valid() const;

public:
  /// @brief Add a sample and map a device timestamp to host time.
  /// @param device Timestamp reported by the L2.
  /// @param host_ns Host system clock time (nanoseconds since the epoch) the packet was received.
  /// @return The device timestamp converted to host time.
  TimeStamp update(const TimeStamp & device, int64_t host_ns);

  /// @brief Host seconds elapsed per device second, used to correct device durations for clock skew.
  /// @return The fitted rate, or 1 until there's enough data to estimate it.
  double rate() const;

  /// @brief The current host system clock time in nanoseconds since the epoch.
  static int64_t host_now_ns()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())
      .count();
  }
};
}  // namespace unilidar2
