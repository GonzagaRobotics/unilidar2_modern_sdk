#include "out_buffer.hpp"

void OutBuffer::finish_active_cloud()
{
    if (active_cloud_ && !active_cloud_->empty())
    {
        if (cloud_buffer_.size() >= BUFFER_CAPACITY)
        {
            cloud_buffer_.pop();
        }

        cloud_buffer_.push(active_cloud_);
        active_cloud_.reset();
    }
}

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

    // TODO: Mid-packet wraps and out of order packets

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
    float theta_b = point_data->com_horizontal_angle_start;
    float theta_c = theta_b;
    float theta_s = point_data->com_horizontal_angle_step;
    float alpha_b = point_data->angle_min;
    float alpha_c = alpha_b;
    float alpha_s = point_data->angle_increment;

    if (last_horizontal_angle_ >= 0.f && theta_b < last_horizontal_angle_)
    {
        finish_active_cloud();

        active_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        active_cloud_->header.stamp = point_data->info.stamp.sec * 1e6 + point_data->info.stamp.nsec / 1e3;
        active_cloud_->header.frame_id = "l2_base_link";
        active_cloud_->is_dense = true;
    }

    last_horizontal_angle_ = theta_b + theta_s * num_pts;

    // If the active cloud is null at this point, it means we need to wait.
    if (!active_cloud_)
    {
        return;
    }

    active_cloud_->points.reserve(active_cloud_->points.size() + num_pts);

    for (int i = 0; i < num_pts; i++, theta_c += theta_s, alpha_c += alpha_s)
    {
        // Skip points of range 0, which are invalid.
        if (point_data->ranges[i] == 0)
        {
            continue;
        }

        // Convert to meters and apply its calibration.
        float range = point_data->param.range_scale * (point_data->ranges[i] + point_data->param.range_bias);

        // Skip points outside the valid range.
        if (range < point_data->range_min || range > point_data->range_max)
        {
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
}

void OutBuffer::add_imu(const ImuData *imu_data)
{
    // IMU data comes in at 250 Hz, but we want 100 Hz. So we need to alternate between keeping every 2 and 3 packets.
    // This averages out to every 2.5 packets, which is close enough to 100 Hz.
    imu_idx_ = (imu_idx_ + 1) % 5;

    if (imu_idx_ == 0 || imu_idx_ == 2)
    {
        imu_buffer_.push(std::make_shared<ImuData>(*imu_data));

        if (imu_buffer_.size() > BUFFER_CAPACITY)
        {
            imu_buffer_.pop();
        }
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
    imu_idx_ = 0;
}
