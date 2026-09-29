#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "unilidar2_sdk/enet_source.hpp"
#include "unilidar2_sdk/lidar.hpp"

class Node : public rclcpp::Node
{
private:
  std::unique_ptr<unilidar2::Lidar> lidar_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

public:
  Node() : rclcpp::Node("unilidar2_node", "l2")
  {
    auto source = std::make_unique<unilidar2::EnetSource>("192.168.1.2", 6201, "192.168.1.62", 6101);
    lidar_ = std::make_unique<unilidar2::Lidar>(std::move(source));

    auto rt_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu", rt_qos);

    // Set work mode and sync time with Lidar before doing anything else

    if (!lidar_->set_work_mode(true)) {
      RCLCPP_ERROR(get_logger(), "Failed to set work mode on Lidar");
      return;
    }

    auto now = get_clock()->now();
    uint32_t sec = now.nanoseconds() / 1000000000;
    uint32_t nsec = now.nanoseconds() % 1000000000;

    if (!lidar_->sync_time(sec, nsec, true)) {
      RCLCPP_ERROR(get_logger(), "Failed to sync time with Lidar");
      return;
    }

    timer_ = create_wall_timer(std::chrono::milliseconds(1), std::bind(&Node::timer_cb, this));
  }

  void timer_cb()
  {
    auto cloud = lidar_->get_cloud();

    if (cloud && !cloud->empty()) {
      sensor_msgs::msg::PointCloud2 cloud_msg;
      pcl::toROSMsg(*cloud, cloud_msg);
      cloud_pub_->publish(cloud_msg);
    }

    auto imu = lidar_->get_imu();

    if (imu) {
      sensor_msgs::msg::Imu imu_msg;
      imu_msg.header.stamp.sec = imu->info.stamp.sec;
      imu_msg.header.stamp.nanosec = imu->info.stamp.nsec;
      imu_msg.header.frame_id = "l2_imu";
      imu_msg.orientation.w = imu->quaternion[0];
      imu_msg.orientation.x = imu->quaternion[1];
      imu_msg.orientation.y = imu->quaternion[2];
      imu_msg.orientation.z = imu->quaternion[3];
      imu_msg.angular_velocity.x = imu->angular_velocity[0];
      imu_msg.angular_velocity.y = imu->angular_velocity[1];
      imu_msg.angular_velocity.z = imu->angular_velocity[2];
      imu_msg.linear_acceleration.x = imu->linear_acceleration[0];
      imu_msg.linear_acceleration.y = imu->linear_acceleration[1];
      imu_msg.linear_acceleration.z = imu->linear_acceleration[2];
      imu_pub_->publish(imu_msg);

      // TODO: Publish TF2
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<Node> node;

  try {
    node = std::make_shared<Node>();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to create node: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  auto executor = rclcpp::executors::SingleThreadedExecutor();
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}