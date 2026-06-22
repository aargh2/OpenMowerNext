#include "hardware_bridge/raspi_system_monitor_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<open_mower_next::hardware_bridge::RaspiSystemMonitorNode>());
  rclcpp::shutdown();
  return 0;
}