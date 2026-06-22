#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include <string>

namespace open_mower_next::hardware_bridge {

class RaspiSystemMonitorNode : public rclcpp::Node {
 public:
  explicit RaspiSystemMonitorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

 private:
  void publish_temperature();
  bool read_cpu_temperature(double & temperature_c) const;

  std::string temperature_path_;
  std::string frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace open_mower_next::hardware_bridge