#include "rclcpp/rclcpp.hpp"

#include "lidar.hpp"

class Node : public rclcpp::Node
{
private:
    Lidar lidar_;

public:
    Node() : rclcpp::Node("unilidar2_node"), lidar_("192.168.1.2", 6201, "192.168.1.62", 6101)
    {
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