#include "unilidar2_sdk/clock_sync.hpp"

#include <cmath>

namespace unilidar2
{
void ClockSync::reset(const TimeStamp & device, int64_t host_ns)
{
  initialized_ = true;
  device_ref_ = device;
  host_ref_ns_ = host_ns;

  weight_ = 0;
  mean_x_ = 0;
  mean_y_ = 0;
  cov_xx_ = 0;
  cov_xy_ = 0;
}

double ClockSync::device_seconds(const TimeStamp & device) const
{
  return (static_cast<double>(device.sec) - device_ref_.sec) +
         (static_cast<double>(device.nsec) - device_ref_.nsec) * 1e-9;
}

bool ClockSync::fit_valid() const { return weight_ > 0 && cov_xx_ / weight_ > MIN_SPREAD * MIN_SPREAD; }

TimeStamp ClockSync::update(const TimeStamp & device, int64_t host_ns)
{
  if (!initialized_) {
    reset(device, host_ns);
  }

  double x = device_seconds(device);
  double y = (host_ns - host_ref_ns_) * 1e-9;

  if (fit_valid()) {
    double predicted = mean_y_ + cov_xy_ / cov_xx_ * (x - mean_x_);
    if (std::fabs(y - predicted) > RESET_ERROR) {
      reset(device, host_ns);
      x = 0;
      y = 0;
    }
  }

  // Exponentially weighted incremental update of the means and co-moments
  weight_ = DECAY * weight_ + 1;
  double dx = x - mean_x_;
  mean_x_ += dx / weight_;
  mean_y_ += (y - mean_y_) / weight_;
  cov_xx_ = DECAY * cov_xx_ + dx * (x - mean_x_);
  cov_xy_ = DECAY * cov_xy_ + dx * (y - mean_y_);

  // Until there's enough spread to estimate the rate, fall back to the receive time
  double mapped = fit_valid() ? mean_y_ + cov_xy_ / cov_xx_ * (x - mean_x_) : y;

  int64_t mapped_ns = host_ref_ns_ + std::llround(mapped * 1e9);

  TimeStamp res;
  res.sec = static_cast<uint32_t>(mapped_ns / 1000000000);
  res.nsec = static_cast<uint32_t>(mapped_ns % 1000000000);
  return res;
}
}  // namespace unilidar2
