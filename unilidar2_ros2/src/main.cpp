#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "tf2_ros/transform_broadcaster.h"

#include "lidar.hpp"

class Node : public rclcpp::Node
{
private:
    Lidar lidar_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr sync_timer_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;

public:
    Node() : rclcpp::Node("unilidar2_node", "l2"), lidar_("192.168.1.2", 6201, "192.168.1.62", 6101), tf_broadcaster_(this)
    {
        auto rt_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

        cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu", rt_qos);

        timer_ = create_wall_timer(std::chrono::milliseconds(1), std::bind(&Node::timer_cb, this));
        sync_timer_ = create_wall_timer(std::chrono::seconds(2), std::bind(&Node::sync_time, this));

        sync_time_wait(true);
        lidar_.set_work_mode(true);
        if (!lidar_.wait_for_ack(1000))
        {
            RCLCPP_ERROR(get_logger(), "Failed to set work mode on Lidar");
        }
    }

    void sync_time()
    {
        sync_time_wait(false);
    }

    void sync_time_wait(bool wait_for_ack)
    {
        auto now = get_clock()->now();
        lidar_.sync_time(now.nanoseconds() / 1000000000, now.nanoseconds() % 1000000000);

        if (wait_for_ack && !lidar_.wait_for_ack(1000))
        {
            RCLCPP_ERROR(get_logger(), "Failed to sync time with Lidar");
        }
    }

    void timer_cb()
    {
        auto cloud = lidar_.get_cloud();

        if (cloud && !cloud->empty())
        {
            sensor_msgs::msg::PointCloud2 cloud_msg;
            pcl::toROSMsg(*cloud, cloud_msg);
            cloud_pub_->publish(cloud_msg);
        }

        auto imu = lidar_.get_imu();

        if (imu)
        {
            sensor_msgs::msg::Imu imu_msg;
            imu_msg.header.stamp.sec = imu->info.stamp.sec;
            imu_msg.header.stamp.nanosec = imu->info.stamp.nsec;
            imu_msg.header.frame_id = "l2_imu";
            // TODO: Does the L2 use WXYZ or XYZW?
            imu_msg.orientation.x = imu->quaternion[0];
            imu_msg.orientation.y = imu->quaternion[1];
            imu_msg.orientation.z = imu->quaternion[2];
            imu_msg.orientation.w = imu->quaternion[3];
            imu_msg.angular_velocity.x = imu->angular_velocity[0];
            imu_msg.angular_velocity.y = imu->angular_velocity[1];
            imu_msg.angular_velocity.z = imu->angular_velocity[2];
            imu_msg.linear_acceleration.x = imu->linear_acceleration[0];
            imu_msg.linear_acceleration.y = imu->linear_acceleration[1];
            imu_msg.linear_acceleration.z = imu->linear_acceleration[2];
            imu_pub_->publish(imu_msg);

            // TODO: Publish TF
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<Node> node;

    try
    {
        node = std::make_shared<Node>();
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to create node: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}