#include "out_buffer.hpp"

L2Cloud OutBuffer::get_cloud()
{
    auto curr = cloud_buffer_.front();
    cloud_buffer_.pop();

    return curr;
}

L2Imu OutBuffer::get_imu()
{
    auto curr = imu_buffer_.front();
    imu_buffer_.pop();

    return curr;
}

void OutBuffer::add_points(const PointData &points)
{
    std::cout << points.point_num << "pts at " << points.com_horizontal_angle_start << " " << points.com_horizontal_angle_step << ", " << points.angle_min << " " << points.angle_increment << std::endl;

    if (!active_cloud_)
    {
        active_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        active_cloud_->header.stamp = points.info.stamp.sec * 1e6 + points.info.stamp.nsec / 1e3;
        active_cloud_->header.frame_id = "lidar";
        active_cloud_->is_dense = true;
        active_cloud_->height = 1;
        active_cloud_->width = 0;
        active_cloud_start_angle_ = points.com_horizontal_angle_start;
    }
    else if (points.com_horizontal_angle_start < active_cloud_start_angle_)
    {
        // TODO: Is this the right condition to determine when a full rotation has been completed?

        active_cloud_->width = active_cloud_->points.size();

        cloud_buffer_.push(active_cloud_);
        if (cloud_buffer_.size() > BUFFER_CAPACITY)
        {
            cloud_buffer_.pop();
        }

        active_cloud_.reset();
    }

    active_cloud_->points.reserve(active_cloud_->points.size() + points.point_num);

    for (uint32_t i = 0; i < points.point_num; i++)
    {
        float range = points.ranges[i] / 1000.0f;
        float azimuth = points.com_horizontal_angle_start + i * points.com_horizontal_angle_step;
        float elevation = points.angle_min + i * points.angle_increment;

        // TODO: Apply calibration parameters to range, azimuth, elevation, etc.
        // TODO: Deskewing based on IMU data and timestamps

        pcl::PointXYZI point;
        point.x = range * cos(elevation) * cos(azimuth);
        point.y = range * cos(elevation) * sin(azimuth);
        point.z = range * sin(elevation);
        point.intensity = points.intensities[i] / 255.0f;

        active_cloud_->points.push_back(point);
    }
}

void OutBuffer::add_imu(const ImuData &imu)
{
    active_imu_.push_back(imu);

    if (active_imu_.size() == 5)
    {
        // TODO: Actually filter the IMU data
        imu_buffer_.push(std::make_shared<ImuData>(active_imu_.back()));
        if (cloud_buffer_.size() > BUFFER_CAPACITY)
        {
            cloud_buffer_.pop();
        }

        active_imu_.clear();
    }
}

void OutBuffer::clear()
{
    while (!cloud_buffer_.empty())
    {
        cloud_buffer_.pop();
    }
    active_cloud_.reset();
    active_cloud_start_angle_ = 0.0f;

    while (!imu_buffer_.empty())
    {
        imu_buffer_.pop();
    }
    active_imu_.clear();
}
