#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "clock_sync.hpp"
#include "messages.hpp"
#include "out_buffer.hpp"
#include "pcl/impl/point_types.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "source.hpp"

namespace unilidar2
{
#define SLEEP_MS(t) std::this_thread::sleep_for(std::chrono::milliseconds(t))

#define PACKET_HEADER(type)                      \
  packet.header.header[0] = FRAME_HEADER_BYTE_0; \
  packet.header.header[1] = FRAME_HEADER_BYTE_1; \
  packet.header.header[2] = FRAME_HEADER_BYTE_2; \
  packet.header.header[3] = FRAME_HEADER_BYTE_3; \
  packet.header.packet_type = type;              \
  packet.header.packet_size = sizeof(packet);

#define PACKET_TAIL                        \
  packet.tail.tail[0] = FRAME_TAIL_BYTE_0; \
  packet.tail.tail[1] = FRAME_TAIL_BYTE_1; \
  auto crc = crc32(0L, Z_NULL, 0);         \
  packet.tail.crc32 = crc32(crc, reinterpret_cast<const uint8_t *>(&packet.data), sizeof(packet.data));

constexpr float TAU = 6.28318530718;

/// @brief One full horizontal sweep of points.
struct Cloud
{
  /// Points of the sweep. header.stamp (microseconds, host time) is the start of the sweep.
  pcl::PointCloud<pcl::PointXYZI> cloud;
  /// Capture time of each point as an offset from header.stamp in microseconds (>= 0), parallel to cloud.points.
  std::vector<float> t_us;
  /// Index of the minor (vertical) sweep each point belongs to, counted from 0 at the start of the cloud. Parallel to
  /// cloud.points.
  std::vector<uint16_t> ring;
};

class Lidar
{
private:
  std::unique_ptr<Source> source_;

  std::unique_ptr<std::thread> rx_thread_;
  std::atomic<bool> running_;

  std::atomic<bool> ack_block_;
  std::mutex mutex_;

  uint8_t buffer_[8192];
  ClockSync clock_sync_;

  float azimuth_rot_ = 0;      // Horizontal rotation accumulated by the active cloud (radians)
  float last_azimuth_ = 0;     // Horizontal start angle of the previous point packet (radians)
  bool have_azimuth_ = false;  // True once last_azimuth_ holds a real angle
  float last_alpha_ = 0;       // Vertical angle of the previous raw point, wrapped to [0, 2pi) (radians)
  bool have_alpha_ = false;    // True once last_alpha_ holds a real angle
  uint16_t minor_sweep_ = 0;   // Minor sweep index within the active cloud
  std::unique_ptr<Cloud> active_cloud_;
  std::vector<int64_t> active_stamps_ns_;  // Absolute host capture time (ns) of each point in the active cloud

  void rx_worker();

  OutBuffer<Cloud> cloud_buffer_;
  OutBuffer<ImuData> imu_buffer_;

  void merge_point_data(const PointData * point_data);

  bool send_packet(const void * data, size_t size, bool blocking);

public:
  Lidar(std::unique_ptr<Source> source);
  Lidar(const Lidar &) = delete;
  Lidar & operator=(const Lidar &) = delete;

  ~Lidar();

  /// @brief Set the work mode of L2.
  /// @param negative_angle If true, expands the scanning range down to be slightly negative.
  /// @param coord_3d If true, scan in 3D mode. If false, scans in 2D mode.
  /// @param imu If true, enables IMU data output.
  /// @param comm_enet If true, enables Ethernet communication. If false, uses UART communication.
  /// @param auto_start If true, starts scanning automatically after power on. If false, requires a command to start scanning.
  /// @return True if the command was sent and acknowledged, false otherwise.
  bool set_work_mode(
    bool negative_angle = false, bool coord_3d = true, bool imu = true, bool comm_enet = true, bool auto_start = true);

  /// @brief Synchronize the L2's internal clock with the provided time.
  /// @param sec The seconds part of the timestamp.
  /// @param nsec The nanoseconds part of the timestamp.
  /// @return True if the command was sent and acknowledged, false otherwise.
  bool sync_time(uint32_t sec, uint32_t nsec);

  /// @brief Reset the L2 device.
  /// @return True if the command was sent and acknowledged, false otherwise.
  bool reset();

  /// @brief Stop the L2's rotation.
  /// @return True if the command was sent and acknowledged, false otherwise.
  bool stop_rotation();

  /// @brief Start the L2's rotation.
  /// @return True if the command was sent and acknowledged, false otherwise.
  bool start_rotation();

  /// @brief Take the oldest buffered sweep. Thread safe.
  /// @return The sweep, or nullptr if none are buffered.
  std::unique_ptr<Cloud> get_cloud();

  /// @brief Take the oldest buffered IMU sample. Thread safe.
  /// @return The sample, or nullptr if none are buffered.
  std::unique_ptr<ImuData> get_imu();
};
}  // namespace unilidar2