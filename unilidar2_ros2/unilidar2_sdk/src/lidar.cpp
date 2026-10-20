#include "unilidar2_sdk/lidar.hpp"

#include <zlib.h>

#include <algorithm>
#include <cmath>
#include <iostream>

#include "unilidar2_sdk/decoding.hpp"
#include "unilidar2_sdk/messages.hpp"

namespace unilidar2
{
void Lidar::rx_worker()
{
  while (running_) {
    std::unique_lock<std::mutex> lock(mutex_);

    size_t buffer_head = 0;
    size_t n;

    try {
      n = source_->get_data(buffer_, sizeof(buffer_));
    } catch (const std::exception & e) {
      std::cerr << "Error in get_data: " << e.what() << std::endl;

      lock.unlock();
      SLEEP_MS(2);
      continue;
    }

    if (n == 0) {
      lock.unlock();
      SLEEP_MS(2);
      continue;
    }

    int64_t rx_time_ns = ClockSync::host_now_ns();

    while (n - buffer_head > 0) {
      DecodeRes res;
      try {
        res = decode_packet(buffer_ + buffer_head, n - buffer_head);
      } catch (const DecodeException & e) {
        std::cerr << "Error decoding packet: " << e.what() << std::endl;
        break;
      }

      buffer_head += res.size + 24;  // Move past the packet header and tail

      if (res.packet_type == ACK_DATA_PACKET_TYPE) {
        AckData * ack = reinterpret_cast<AckData *>(res.data.get());
        std::cout << "Received ACK: " << ack_packet_to_string(ack) << std::endl;

        ack_block_ = false;  // Clear the ack_block_ flag to indicate that the ACK has been received
        continue;            // No further processing needed for ACK packets
      } else if (ack_block_) {
        continue;  // If we're waiting for an ACK, ignore other packet types
      }

      if (res.packet_type == IMU_DATA_PACKET_TYPE) {
        std::unique_ptr<ImuData> imu_data(reinterpret_cast<ImuData *>(res.data.release()));
        imu_data->info.stamp = clock_sync_.update(imu_data->info.stamp, rx_time_ns);

        imu_buffer_.push(std::move(imu_data));
      } else if (res.packet_type == POINT_DATA_PACKET_TYPE) {
        std::unique_ptr<PointData> point_data(reinterpret_cast<PointData *>(res.data.release()));
        point_data->info.stamp = clock_sync_.update(point_data->info.stamp, rx_time_ns);
        merge_point_data(point_data.get());
      } else {
        std::cerr << "Received unexpected packet type: " << res.packet_type << std::endl;
      }
    }

    // Data was available, so immediately check for more instead of sleeping. Sleeping after every datagram caps the
    // read rate below the L2's packet rate and lets the socket buffer overflow.
    lock.unlock();
  }
}

void Lidar::merge_point_data(const PointData * point_data)
{
  // FIXME: Can cause incorrect clouds if data arrives out of order in the right circumstance

  // Intermediate calibration values
  float sin_beta = sin(point_data->param.beta_angle);
  float cos_beta = cos(point_data->param.beta_angle);
  float sin_xi = sin(point_data->param.xi_angle);
  float cos_xi = cos(point_data->param.xi_angle);
  float cos_beta_sin_xi = cos_beta * sin_xi;
  float sin_beta_cos_xi = sin_beta * cos_xi;
  float cos_beta_cos_xi = cos_beta * cos_xi;
  float sin_beta_sin_xi = sin_beta * sin_xi;

  int num_pts = point_data->point_num;
  float theta_c = point_data->com_horizontal_angle_start;
  float theta_s = point_data->com_horizontal_angle_step;
  float alpha_b = point_data->angle_min;
  float alpha_c = alpha_b;
  float alpha_s = point_data->angle_increment;

  // Each point is sampled time_increment after the previous one, starting at the packet stamp. time_increment is
  // measured by the L2's clock, so scale it by the fitted clock rate to convert it to host time.
  int64_t packet_ns = int64_t(point_data->info.stamp.sec) * 1000000000 + point_data->info.stamp.nsec;
  int64_t time_step_ns = std::llround(point_data->time_increment * clock_sync_.rate() * 1e9);

  if (!active_cloud_) {
    active_cloud_ = std::make_unique<Cloud>();
    active_cloud_->cloud.header.frame_id = "lidar_link";
    active_cloud_->cloud.is_dense = true;
    active_cloud_->cloud.height = 1;
    active_cloud_->cloud.width = 0;
    active_stamps_ns_.clear();
    minor_sweep_ = 0;
  }

  // Track the horizontal rotation using the start angle of each packet. The difference is wrapped into [-pi, pi)
  // so the jump where the angle wraps around 2pi -> 0 still counts as a small forward step.
  if (have_azimuth_) {
    azimuth_rot_ += std::remainder(theta_c - last_azimuth_, TAU);
  }
  last_azimuth_ = theta_c;
  have_azimuth_ = true;

  active_cloud_->cloud.reserve(active_cloud_->cloud.size() + num_pts);
  active_stamps_ns_.reserve(active_stamps_ns_.size() + num_pts);
  active_cloud_->ring.reserve(active_cloud_->ring.size() + num_pts);

  for (int i = 0; i < num_pts; i++, theta_c += theta_s, alpha_c += alpha_s) {
    // A new minor sweep starts whenever the wrapped vertical angle steps backwards. Checked on every raw point,
    // including invalid ones, so dropped points cannot hide a wrap.
    float alpha_wrapped = alpha_c - TAU * std::floor(alpha_c / TAU);
    if (have_alpha_ && alpha_wrapped < last_alpha_) {
      minor_sweep_++;
    }
    last_alpha_ = alpha_wrapped;
    have_alpha_ = true;

    // Skip points of range 0, which are invalid.
    if (point_data->ranges[i] == 0) {
      continue;
    }

    // Convert to meters and apply its calibration.
    float range = point_data->param.range_scale * (point_data->ranges[i] + point_data->param.range_bias);

    // Skip points outside the valid range.
    if (range < point_data->range_min || range > point_data->range_max) {
      continue;
    }

    // Transform to cartesian coordinates and add in calibration
    float sin_theta = sin(theta_c);
    float cos_theta = cos(theta_c);
    float sin_alpha = sin(alpha_c);
    float cos_alpha = cos(alpha_c);

    float A = (-cos_beta_sin_xi * sin_beta_cos_xi * sin_alpha) * range + point_data->param.b_axis_dist;
    float B = cos_alpha * cos_xi * range;
    float C = (sin_beta_sin_xi + cos_beta_cos_xi * sin_alpha) * range;

    pcl::PointXYZI point;
    point.x = A * cos_theta - B * sin_theta;
    point.y = A * sin_theta + B * cos_theta;
    point.z = C + point_data->param.a_axis_dist;
    point.intensity = point_data->intensities[i] / 255.0f;
    active_cloud_->cloud.push_back(point);
    active_stamps_ns_.push_back(packet_ns + i * time_step_ns);
    active_cloud_->ring.push_back(minor_sweep_);
  }

  if (std::fabs(azimuth_rot_) >= TAU) {
    // Carry the overshoot into the next cloud so cloud boundaries don't drift
    azimuth_rot_ = std::fmod(azimuth_rot_, TAU);

    // Stamp the cloud at the earliest point so every offset is >= 0, even if packets arrived out of order. PCL stamps
    // are in microseconds, so truncate the start and measure offsets from the truncated value.
    int64_t start_ns =
      active_stamps_ns_.empty() ? packet_ns : *std::min_element(active_stamps_ns_.begin(), active_stamps_ns_.end());
    uint64_t start_us = start_ns / 1000;
    active_cloud_->cloud.header.stamp = start_us;

    int64_t start_trunc_ns = int64_t(start_us) * 1000;
    active_cloud_->t_us.reserve(active_stamps_ns_.size());
    for (int64_t stamp_ns : active_stamps_ns_) {
      active_cloud_->t_us.push_back(static_cast<float>(stamp_ns - start_trunc_ns) * 1e-3f);
    }
    active_stamps_ns_.clear();

    cloud_buffer_.push(std::move(active_cloud_));
  }
}

bool Lidar::send_packet(const void * data, size_t size, bool blocking)
{
  std::unique_lock<std::mutex> lock(mutex_);

  if (blocking) {
    ack_block_ = true;
    // Blocking calls indicate that the data we have is now stale
    active_cloud_ = nullptr;
    cloud_buffer_.clear();
    imu_buffer_.clear();
  }

  try {
    source_->send_data(static_cast<const uint8_t *>(data), size);
  } catch (const std::exception & e) {
    std::cerr << "Error sending packet: " << e.what() << std::endl;

    ack_block_ = false;
    return false;
  }

  // Unlock the mutex before waiting for the ACK to avoid deadlocks
  lock.unlock();

  if (!blocking) {
    return true;
  }

  auto start_t = std::chrono::steady_clock::now();

  // Wait for ack_block_ to be cleared by the rx_worker thread
  while (ack_block_) {
    auto elapsed = std::chrono::steady_clock::now() - start_t;
    if (elapsed > std::chrono::seconds(1)) {
      ack_block_ = false;
      return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  return true;
}

Lidar::Lidar(std::unique_ptr<Source> source) : source_(std::move(source))
{
  running_ = true;
  rx_thread_ = std::make_unique<std::thread>(&Lidar::rx_worker, this);
}

Lidar::~Lidar()
{
  running_ = false;
  rx_thread_->join();
}

std::unique_ptr<Cloud> Lidar::get_cloud()
{
  return cloud_buffer_.pop();
}

std::unique_ptr<ImuData> Lidar::get_imu()
{
  return imu_buffer_.pop();
}

bool Lidar::set_work_mode(bool negative_angle, bool coord_3d, bool imu, bool comm_enet, bool auto_start)
{
  WorkModeConfigPacket packet{};
  PACKET_HEADER(WORK_MODE_CONFIG_PACKET_TYPE)

  packet.data.mode |= negative_angle ? 1 : 0;
  packet.data.mode |= !coord_3d ? 2 : 0;
  packet.data.mode |= !imu ? 4 : 0;
  packet.data.mode |= !comm_enet ? 8 : 0;
  packet.data.mode |= !auto_start ? 16 : 0;
  // Rest is reserved

  PACKET_TAIL

  return send_packet(&packet, sizeof(packet), true);
}

bool Lidar::sync_time(uint32_t sec, uint32_t nsec)
{
  TimeStampPacket packet{};
  PACKET_HEADER(TIME_STAMP_PACKET_TYPE)

  packet.data.sec = sec;
  packet.data.nsec = nsec;

  PACKET_TAIL

  return send_packet(&packet, sizeof(packet), true);
}

bool Lidar::reset()
{
  UserCtrlCmdPacket packet{};
  PACKET_HEADER(USER_CMD_PACKET_TYPE)

  packet.data.type = USER_CMD_RESET_TYPE;
  packet.data.value = 0;

  PACKET_TAIL

  return send_packet(&packet, sizeof(packet), true);
}

bool Lidar::stop_rotation()
{
  UserCtrlCmdPacket packet{};
  PACKET_HEADER(USER_CMD_PACKET_TYPE)

  packet.data.type = USER_CMD_STANDBY_TYPE;
  packet.data.value = 1;

  PACKET_TAIL

  return send_packet(&packet, sizeof(packet), true);
}

bool Lidar::start_rotation()
{
  UserCtrlCmdPacket packet{};
  PACKET_HEADER(USER_CMD_PACKET_TYPE)

  packet.data.type = USER_CMD_STANDBY_TYPE;
  packet.data.value = 0;

  PACKET_TAIL

  return send_packet(&packet, sizeof(packet), true);
}

}  // namespace unilidar2