#pragma once

#include <vector>
#include <queue>
#include <memory>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "messages.hpp"

using L2Cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr;
using L2Imu = std::shared_ptr<ImuData>;

/// @brief A buffer that processes incoming raw data and outputs usable point clouds and IMU data. Not thread-safe.
class OutBuffer
{
private:
    const size_t BUFFER_CAPACITY = 32;

    // A queue of point clouds that have been fully filled in.
    std::queue<L2Cloud> cloud_buffer_;
    // The cloud that is actively being filled with points. It is not part of the buffer yet.
    L2Cloud active_cloud_;
    // The horizontal angle the last batch of points started at.
    float last_horizontal_angle_ = -1.f;

    // A queue of IMU data after downsampling.
    std::queue<L2Imu> imu_buffer_;
    // Index of IMU packets to keep track for downsampling.
    int imu_idx_;

    void finish_active_cloud();

public:
    OutBuffer() = default;
    OutBuffer(size_t capacity) : BUFFER_CAPACITY(capacity) {};

    L2Cloud get_cloud();
    L2Imu get_imu();

    void add_points(const PointData *point_data);
    void add_imu(const ImuData *imu_data);

    void clear();
};