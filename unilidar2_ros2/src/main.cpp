#include <chrono>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "source_params.hpp"
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
    lidar_ = std::make_unique<unilidar2::Lidar>(make_source(*this));

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 5);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS());

    timer_ = create_wall_timer(std::chrono::milliseconds(1), std::bind(&Node::timer_cb, this));
  }

  /// Publish a sweep with fields x, y, z, intensity, time, ring. The header stamp is the start of the sweep, time is each
  /// point's offset from it in seconds (float32, >= 0), and ring is the point's minor sweep index (uint16), matching the
  /// Velodyne layout FAST-LIO2 and similar expect.
  void publish_cloud(const unilidar2::Cloud & cloud)
  {
    // PCL stamps are in microseconds
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp.sec = static_cast<int32_t>(cloud.cloud.header.stamp / 1000000);
    msg.header.stamp.nanosec = static_cast<uint32_t>(cloud.cloud.header.stamp % 1000000 * 1000);
    msg.header.frame_id = cloud.cloud.header.frame_id;

    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2Fields(
      6, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32, "z", 1,
      sensor_msgs::msg::PointField::FLOAT32, "intensity", 1, sensor_msgs::msg::PointField::FLOAT32, "time", 1,
      sensor_msgs::msg::PointField::FLOAT32, "ring", 1, sensor_msgs::msg::PointField::UINT16);
    modifier.resize(cloud.cloud.size());
    msg.height = 1;
    msg.is_dense = true;

    sensor_msgs::PointCloud2Iterator<float> it_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(msg, "z");
    sensor_msgs::PointCloud2Iterator<float> it_i(msg, "intensity");
    sensor_msgs::PointCloud2Iterator<float> it_t(msg, "time");
    sensor_msgs::PointCloud2Iterator<uint16_t> it_r(msg, "ring");

    for (size_t i = 0; i < cloud.cloud.size(); i++, ++it_x, ++it_y, ++it_z, ++it_i, ++it_t, ++it_r) {
      const auto & p = cloud.cloud.points[i];
      *it_x = p.x;
      *it_y = p.y;
      *it_z = p.z;
      *it_i = p.intensity;
      *it_t = cloud.t_us[i] * 1e-6f;
      *it_r = cloud.ring[i];
    }

    cloud_pub_->publish(msg);
  }

  void timer_cb()
  {
    auto cloud = lidar_->get_cloud();

    if (cloud && !cloud->cloud.empty()) {
      publish_cloud(*cloud);
    }

    auto imu = lidar_->get_imu();

    if (imu) {
      sensor_msgs::msg::Imu imu_msg;
      imu_msg.header.stamp.sec = imu->info.stamp.sec;
      imu_msg.header.stamp.nanosec = imu->info.stamp.nsec;
      imu_msg.header.frame_id = "imu_link";
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