#include "unilidar2_sdk/lidar.hpp"

#include <zlib.h>

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
        imu_buffer_.push(std::move(imu_data));
      } else if (res.packet_type == POINT_DATA_PACKET_TYPE) {
        std::unique_ptr<PointData> point_data(reinterpret_cast<PointData *>(res.data.release()));
        merge_point_data(point_data.get());
      } else {
        std::cerr << "Received unexpected packet type: " << res.packet_type << std::endl;
      }
    }

    lock.unlock();
    SLEEP_MS(2);
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

  if (!active_cloud_) {
    active_cloud_ = std::make_unique<pcl::PointCloud<pcl::PointXYZI>>();
    active_cloud_->header.frame_id = "lidar_link";
    active_cloud_->is_dense = false;
    active_cloud_->height = 1;
    active_cloud_->width = 0;

    azimuth_rot_ = 0;
    if (last_azimuth_ < 0) {
      last_azimuth_ = theta_c;
    }
  }

  active_cloud_->reserve(active_cloud_->size() + num_pts);

  for (int i = 0; i < num_pts; i++, theta_c += theta_s, alpha_c += alpha_s) {
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
    active_cloud_->push_back(point);
  }

  azimuth_rot_ += (theta_c - last_azimuth_) + theta_s * num_pts;
  last_azimuth_ = theta_c;

  if (azimuth_rot_ >= TAU) {
    active_cloud_->header.stamp = point_data->info.stamp.sec * 1000000000ULL + point_data->info.stamp.nsec;
    cloud_buffer_.push(std::move(active_cloud_));
  }
}

bool Lidar::send_packet(const void * data, size_t size, bool blocking)
{
  std::unique_lock<std::mutex> lock(mutex_);

  if (blocking) {
    ack_block_ = true;
    // Blocking calls indicate that the data we have is now stale
    imu_buffer_.clear();
    cloud_buffer_.clear();
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