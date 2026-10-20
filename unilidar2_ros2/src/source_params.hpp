#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>

#include "unilidar2_sdk/enet_source.hpp"
#include "unilidar2_sdk/source.hpp"
#include "unilidar2_sdk/uart_source.hpp"

/// @brief Declare a parameter that can only be set at startup (launch file / command line overrides).
template <typename T>
T declare_start_param(
  rclcpp::Node & node, const std::string & name, const T & default_value, const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  descriptor.read_only = true;
  return node.declare_parameter<T>(name, default_value, descriptor);
}

inline uint16_t declare_port_param(
  rclcpp::Node & node, const std::string & name, int64_t default_value, const std::string & description)
{
  auto port = declare_start_param<int64_t>(node, name, default_value, description);
  if (port < 1 || port > 65535) {
    throw std::invalid_argument("Parameter '" + name + "' must be in range [1, 65535], got " + std::to_string(port));
  }
  return static_cast<uint16_t>(port);
}

/// @brief Declare the connection parameters and create the selected source.
inline std::unique_ptr<unilidar2::Source> make_source(rclcpp::Node & node)
{
  auto connection =
    declare_start_param<std::string>(node, "connection", "enet", "Connection to the L2: 'enet' or 'uart'");

  auto local_ip = declare_start_param<std::string>(node, "enet.local_ip", "192.168.1.2", "IP address of this host");
  auto local_port = declare_port_param(node, "enet.local_port", 6201, "UDP port on this host");
  auto remote_ip = declare_start_param<std::string>(node, "enet.remote_ip", "192.168.1.62", "IP address of the L2");
  auto remote_port = declare_port_param(node, "enet.remote_port", 6101, "UDP port on the L2");

  auto uart_port = declare_start_param<std::string>(node, "uart.port", "/dev/ttyACM0", "Serial device of the L2");

  if (connection == "enet") {
    RCLCPP_INFO(
      node.get_logger(), "Connecting over ethernet: %s:%u -> %s:%u", local_ip.c_str(), local_port, remote_ip.c_str(),
      remote_port);
    return std::make_unique<unilidar2::EnetSource>(local_ip, local_port, remote_ip, remote_port);
  }

  if (connection == "uart") {
    RCLCPP_INFO(node.get_logger(), "Connecting over UART: %s", uart_port.c_str());
    return std::make_unique<unilidar2::UartSource>(uart_port);
  }

  throw std::invalid_argument("Parameter 'connection' must be 'enet' or 'uart', got '" + connection + "'");
}
