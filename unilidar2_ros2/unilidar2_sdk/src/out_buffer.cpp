#include "out_buffer.hpp"

L2Cloud OutBuffer::get_cloud()
{
    if (cloud_buffer_.empty())
    {
        return nullptr;
    }

    auto curr = cloud_buffer_.front();
    cloud_buffer_.pop();

    return curr;
}

L2Imu OutBuffer::get_imu()
{
    if (imu_buffer_.empty())
    {
        return nullptr;
    }

    auto curr = imu_buffer_.front();
    imu_buffer_.pop();

    return curr;
}

void OutBuffer::add_points(const PointData *point_data)
{
    // The horizontal angle goes from 0 to 2pi, which we can use to determine when a full rotation has been completed.

    if (last_seq_ >= 0 && point_data->info.seq != last_seq_ + 1)
    {
        // std::cout << "Warning: PointData sequence number jumped from " << last_seq_ << " to " << point_data->info.seq << std::endl;
    }

    if (point_data->com_horizontal_angle_start + point_data->com_horizontal_angle_step * point_data->point_num < last_horizontal_angle_)
    {
        cloud_buffer_.push(active_cloud_);
        if (cloud_buffer_.size() > BUFFER_CAPACITY)
        {
            cloud_buffer_.pop();
        }

        active_cloud_.reset();
    }

    // if (last_horizontal_angle_ < 0.f)
    // {
    //     std::cout << "Lidar state at init:" << std::endl
    //               << "  sys_rotation_period: " << point_data->state.sys_rotation_period << std::endl
    //               << "  com_rotation_period: " << point_data->state.com_rotation_period << std::endl
    //               << "  dirty_index: " << point_data->state.dirty_index << std::endl
    //               << "  packet_lost_up: " << point_data->state.packet_lost_up << std::endl
    //               << "  packet_lost_down: " << point_data->state.packet_lost_down << std::endl
    //               << "  apd_temperature: " << point_data->state.apd_temperature << std::endl
    //               << "  apd_voltage: " << point_data->state.apd_voltage << std::endl
    //               << "  laser_voltage: " << point_data->state.laser_voltage << std::endl
    //               << "  imu_temperature: " << point_data->state.imu_temperature << std::endl;
    //     std::cout << "Lidar calibration parameters:" << std::endl
    //               << "  a_axis_dist: " << point_data->param.a_axis_dist << std::endl
    //               << "  b_axis_dist: " << point_data->param.b_axis_dist << std::endl
    //               << "  theta_angle_bias: " << point_data->param.theta_angle_bias << std::endl
    //               << "  alpha_angle_bias: " << point_data->param.alpha_angle_bias << std::endl
    //               << "  beta_angle: " << point_data->param.beta_angle << std::endl
    //               << "  xi_angle: " << point_data->param.xi_angle << std::endl
    //               << "  range_bias: " << point_data->param.range_bias << std::endl
    //               << "  range_scale: " << point_data->param.range_scale << std::endl;
    // }

    last_horizontal_angle_ = point_data->com_horizontal_angle_start + point_data->com_horizontal_angle_step * point_data->point_num;
    last_seq_ = point_data->info.seq;

    if (!active_cloud_)
    {
        active_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        active_cloud_->header.stamp = point_data->info.stamp.sec * 1e6 + point_data->info.stamp.nsec / 1e3;
        active_cloud_->header.frame_id = "l2_base_link";
        active_cloud_->is_dense = true;
    }

    active_cloud_->points.reserve(active_cloud_->points.size() + point_data->point_num);

    int num_zero = 0;
    for (uint32_t i = 0; i < point_data->point_num; i++)
    {
        float range = point_data->ranges[i] / 1000.0f;

        if (range < point_data->range_min || range > point_data->range_max)
        {
            num_zero++;
            continue;
        }

        float azimuth = point_data->com_horizontal_angle_start + i * point_data->com_horizontal_angle_step;
        float elevation = point_data->angle_min + i * point_data->angle_increment;

        // TODO: Apply calibration parameters to range, azimuth, elevation, etc.
        // TODO: Deskewing based on IMU data and timestamps

        pcl::PointXYZI point;
        point.x = range * cos(elevation) * cos(azimuth);
        point.y = range * cos(elevation) * sin(azimuth);
        point.z = range * sin(elevation);
        point.intensity = point_data->intensities[i] / 255.0f;

        active_cloud_->push_back(point);
    }

    if (num_zero > 0)
    {
        // std::cout << "Warning: " << num_zero << " points with zero range in this packet" << std::endl;
    }
}

void OutBuffer::add_imu(const ImuData *imu_data)
{
    // active_imu_.push_back(*imu_data);

    // if (active_imu_.size() > 5)
    // {
    //     // TODO: Filtering?
    //     ImuData filtered_imu{};
    //     filtered_imu.info = active_imu_.back().info;

    //     if (imu_buffer_.size() > BUFFER_CAPACITY)
    //     {
    //         imu_buffer_.pop();
    //     }

    //     active_imu_.pop();
    // }
    imu_buffer_.push(std::make_shared<ImuData>(*imu_data));

    if (imu_buffer_.size() > BUFFER_CAPACITY)
    {
        imu_buffer_.pop();
    }
}

void OutBuffer::clear()
{
    while (!cloud_buffer_.empty())
    {
        cloud_buffer_.pop();
    }
    active_cloud_.reset();
    last_horizontal_angle_ = -1.0f;

    while (!imu_buffer_.empty())
    {
        imu_buffer_.pop();
    }
    active_imu_.clear();
}
