#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

#include "messages.hpp"
#include "out_buffer.hpp"
#include "pcl/impl/point_types.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "source.hpp"

namespace unilidar2
{
#define SLEEP_2 std::this_thread::sleep_for(std::chrono::milliseconds(2))

constexpr float kTau = 6.28318530718;

class Lidar
{
private:
  std::unique_ptr<Source> source_;

  std::unique_ptr<std::thread> rx_thread_;
  std::atomic<bool> running_;

  std::atomic<bool> ack_block_;
  std::mutex mutex_;

  uint8_t buffer_[8192];

  float azimuth_rot_;
  float last_azimuth_ = -1;
  std::unique_ptr<pcl::PointCloud<pcl::PointXYZI>> active_cloud_;

  void rx_worker();

  OutBuffer<pcl::PointCloud<pcl::PointXYZI>> cloud_buffer_;
  OutBuffer<ImuData> imu_buffer_;

  void merge_point_data(const PointData * point_data);

  bool send_packet(const void * data, size_t size, bool blocking);

public:
  Lidar(std::unique_ptr<Source> source);
  Lidar(const Lidar &) = delete;
  Lidar & operator=(const Lidar &) = delete;

  ~Lidar();

  /// Set the work mode of the Lidar.
  bool set_work_mode(bool negative_angle);
  bool sync_time(uint32_t sec, uint32_t nsec, bool block = false);

  std::unique_ptr<pcl::PointCloud<pcl::PointXYZI>> get_cloud() { return cloud_buffer_.pop(); }
  std::unique_ptr<ImuData> get_imu() { return imu_buffer_.pop(); }
};
}  // namespace unilidar2