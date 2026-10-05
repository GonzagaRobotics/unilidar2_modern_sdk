#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "source_params.hpp"
#include "unilidar2_sdk/lidar.hpp"

int set_mode(rclcpp::Node & node)
{
  auto logger = node.get_logger();

  auto negative_angle =
    declare_start_param<bool>(node, "mode.negative_angle", false, "Expand the scan range to be slightly negative");
  auto coord_3d = declare_start_param<bool>(node, "mode.coord_3d", true, "Scan in 3D mode (false for 2D)");
  auto imu = declare_start_param<bool>(node, "mode.imu", true, "Enable IMU data output");
  auto comm_enet =
    declare_start_param<bool>(node, "mode.comm_enet", true, "Communicate over ethernet after reset (false for UART)");
  auto auto_start = declare_start_param<bool>(
    node, "mode.auto_start", true, "Start scanning on power on (false to require a start command)");

  auto lidar = unilidar2::Lidar(make_source(node));

  RCLCPP_INFO(
    logger, "Setting work mode: negative_angle=%d coord_3d=%d imu=%d comm_enet=%d auto_start=%d", negative_angle,
    coord_3d, imu, comm_enet, auto_start);

  if (!lidar.set_work_mode(negative_angle, coord_3d, imu, comm_enet, auto_start)) {
    RCLCPP_ERROR(logger, "Fault: Work mode");
    return 1;
  }

  rclcpp::sleep_for(std::chrono::milliseconds(500));

  if (!lidar.reset()) {
    RCLCPP_ERROR(logger, "Fault: Reset");
    return 1;
  }

  // Short delay to allow the lidar to reset before starting rotation
  rclcpp::sleep_for(std::chrono::seconds(10));
  return 0;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<rclcpp::Node>("unilidar2_set_mode", "l2");
    int result = set_mode(*node);

    if (result != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to set mode");
      rclcpp::shutdown();
      return result;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception occurred: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
