#include <rclcpp/rclcpp.hpp>

#include "hardware_bridge/mainboard_serial_bridge_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<open_mower_next::hardware_bridge::MainboardSerialBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
