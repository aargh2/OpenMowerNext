#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <open_mower_next/msg/ui_button_event.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "hardware_bridge/protocol.hpp"

namespace open_mower_next::hardware_bridge {

class MainboardSerialBridgeNode : public rclcpp::Node {
 public:
  explicit MainboardSerialBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MainboardSerialBridgeNode() override;

 private:
  void read_loop();
  bool open_serial();
  void close_serial();
  void configure_serial(int fd) const;
  void process_frame(const std::vector<uint8_t> & encoded_frame);
  void handle_status(const std::vector<uint8_t> & packet);
  void handle_imu(const std::vector<uint8_t> & packet);
  void send_heartbeat();
  bool write_packet(const uint8_t * data, size_t size);

  int serial_fd_{-1};
  std::mutex serial_mutex_;
  std::thread read_thread_;
  std::atomic_bool running_{false};
  std::atomic_bool low_level_emergency_{false};
  std::atomic_bool emergency_requested_{false};
  std::atomic_bool release_requested_{false};

  std::string serial_port_;
  int baudrate_{115200};
  double reconnect_period_s_{5.0};
  double charger_present_voltage_threshold_{10.0};
  std::string imu_frame_id_{"imu"};
  std::string battery_frame_id_{"base_link"};

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr charger_present_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rain_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr charge_voltage_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<open_mower_next::msg::UiButtonEvent>::SharedPtr ui_button_event_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr emergency_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_emergency_service_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace open_mower_next::hardware_bridge
