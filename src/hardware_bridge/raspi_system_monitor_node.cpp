#include "hardware_bridge/raspi_system_monitor_node.hpp"

#include <algorithm>
#include <chrono>
#include <fstream>

namespace open_mower_next::hardware_bridge {

RaspiSystemMonitorNode::RaspiSystemMonitorNode(const rclcpp::NodeOptions & options)
    : Node("raspi_system_monitor", options) {
  temperature_path_ =
      declare_parameter<std::string>("temperature_path", "/sys/class/thermal/thermal_zone0/temp");
  frame_id_ = declare_parameter<std::string>("frame_id", "raspi5");
  const auto topic =
      declare_parameter<std::string>("temperature_topic", "/hardware/raspi/cpu_temperature");
  const auto publish_rate_hz = declare_parameter<double>("publish_rate_hz", 1.0);

  temperature_pub_ = create_publisher<sensor_msgs::msg::Temperature>(topic, 10);
  const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, publish_rate_hz));
  timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { publish_temperature(); });
}

void RaspiSystemMonitorNode::publish_temperature() {
  double temperature_c = 0.0;
  if (!read_cpu_temperature(temperature_c)) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000, "Cannot read Raspberry Pi temperature from %s",
        temperature_path_.c_str());
    return;
  }

  sensor_msgs::msg::Temperature msg;
  msg.header.stamp = now();
  msg.header.frame_id = frame_id_;
  msg.temperature = temperature_c;
  msg.variance = 0.0;
  temperature_pub_->publish(msg);
}

bool RaspiSystemMonitorNode::read_cpu_temperature(double & temperature_c) const {
  std::ifstream input(temperature_path_);
  long millidegrees = 0;
  if (!(input >> millidegrees)) {
    return false;
  }
  temperature_c = static_cast<double>(millidegrees) / 1000.0;
  return true;
}

}  // namespace open_mower_next::hardware_bridge