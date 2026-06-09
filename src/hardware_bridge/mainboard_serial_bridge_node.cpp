#include "hardware_bridge/mainboard_serial_bridge_node.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>

namespace open_mower_next::hardware_bridge {
namespace {

speed_t to_baudrate(int baudrate)
{
  switch (baudrate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    default:
      throw std::runtime_error("Unsupported serial baudrate: " + std::to_string(baudrate));
  }
}

template <typename T>
T parse_packet(const std::vector<uint8_t> & packet)
{
  T parsed{};
  std::memcpy(&parsed, packet.data(), sizeof(T));
  return parsed;
}

}  // namespace

MainboardSerialBridgeNode::MainboardSerialBridgeNode(const rclcpp::NodeOptions & options)
    : Node("mainboard_serial_bridge", options)
{
  serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyAMA0");
  baudrate_ = declare_parameter<int>("baudrate", 115200);
  reconnect_period_s_ = declare_parameter<double>("reconnect_period_s", 5.0);
  charger_present_voltage_threshold_ =
      declare_parameter<double>("charger_present_voltage_threshold", 10.0);
  imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "imu");
  battery_frame_id_ = declare_parameter<std::string>("battery_frame_id", "base_link");
  const auto heartbeat_rate_hz = declare_parameter<double>("heartbeat_rate_hz", 50.0);

  charger_present_pub_ = create_publisher<std_msgs::msg::Bool>("/power/charger_present", 10);
  emergency_pub_ = create_publisher<std_msgs::msg::Bool>("/hardware/emergency", 10);
  rain_pub_ = create_publisher<std_msgs::msg::Bool>("/hardware/rain", 10);
  charge_voltage_pub_ = create_publisher<std_msgs::msg::Float32>("/power/charge_voltage", 10);
  battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("/power", 10);
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 50);

  emergency_service_ = create_service<std_srvs::srv::SetBool>(
      "/hardware/set_emergency",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
             std_srvs::srv::SetBool::Response::SharedPtr response) {
        emergency_requested_.store(request->data);
        if (!request->data) {
          release_requested_.store(true);
        }
        response->success = true;
        response->message = request->data ? "Emergency request latched" : "Emergency release requested";
      });

  clear_emergency_service_ = create_service<std_srvs::srv::Trigger>(
      "/hardware/clear_emergency",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr response) {
        emergency_requested_.store(false);
        release_requested_.store(true);
        response->success = true;
        response->message = "Emergency release requested";
      });

  const auto heartbeat_period =
      std::chrono::duration<double>(1.0 / std::max(1.0, heartbeat_rate_hz));
  heartbeat_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(heartbeat_period),
      [this]() { send_heartbeat(); });

  running_.store(true);
  read_thread_ = std::thread([this]() { read_loop(); });
}

MainboardSerialBridgeNode::~MainboardSerialBridgeNode()
{
  running_.store(false);
  close_serial();
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
}

bool MainboardSerialBridgeNode::open_serial()
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (serial_fd_ >= 0) {
    return true;
  }

  const int fd = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Cannot open serial port %s: %s", serial_port_.c_str(),
        std::strerror(errno));
    return false;
  }

  try {
    configure_serial(fd);
  } catch (const std::exception & ex) {
    ::close(fd);
    RCLCPP_ERROR(get_logger(), "Failed to configure serial port %s: %s", serial_port_.c_str(), ex.what());
    return false;
  }

  serial_fd_ = fd;
  RCLCPP_INFO(get_logger(), "Connected to mainboard on %s at %d baud", serial_port_.c_str(), baudrate_);
  return true;
}

void MainboardSerialBridgeNode::close_serial()
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

void MainboardSerialBridgeNode::configure_serial(int fd) const
{
  termios tty{};
  if (tcgetattr(fd, &tty) != 0) {
    throw std::runtime_error(std::strerror(errno));
  }

  cfmakeraw(&tty);
  tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
  tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
  tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
  tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
  tty.c_cflag |= CS8;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  const auto baud = to_baudrate(baudrate_);
  cfsetispeed(&tty, baud);
  cfsetospeed(&tty, baud);

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    throw std::runtime_error(std::strerror(errno));
  }
}

void MainboardSerialBridgeNode::read_loop()
{
  std::vector<uint8_t> frame;
  frame.reserve(256);

  while (running_.load()) {
    if (!open_serial()) {
      std::this_thread::sleep_for(std::chrono::duration<double>(reconnect_period_s_));
      continue;
    }

    int fd = -1;
    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      fd = serial_fd_;
    }

    pollfd pfd{};
    pfd.fd = fd;
    pfd.events = POLLIN;
    const int poll_result = ::poll(&pfd, 1, 200);
    if (poll_result < 0) {
      if (errno != EINTR) {
        RCLCPP_WARN(get_logger(), "Serial poll failed: %s", std::strerror(errno));
        close_serial();
      }
      continue;
    }
    if (poll_result == 0) {
      continue;
    }
    if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      RCLCPP_WARN(get_logger(), "Serial port disconnected");
      close_serial();
      continue;
    }

    uint8_t byte = 0;
    const ssize_t bytes_read = ::read(fd, &byte, 1);
    if (bytes_read < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        RCLCPP_WARN(get_logger(), "Serial read failed: %s", std::strerror(errno));
        close_serial();
      }
      continue;
    }
    if (bytes_read == 0) {
      continue;
    }

    if (byte == 0) {
      if (!frame.empty()) {
        process_frame(frame);
        frame.clear();
      }
      continue;
    }

    if (frame.size() >= 512) {
      RCLCPP_WARN(get_logger(), "Dropping oversized serial frame");
      frame.clear();
      continue;
    }
    frame.push_back(byte);
  }
}

void MainboardSerialBridgeNode::process_frame(const std::vector<uint8_t> & encoded_frame)
{
  const auto decoded = cobs_decode(encoded_frame);
  if (!decoded.has_value()) {
    RCLCPP_WARN(get_logger(), "Dropping invalid COBS frame");
    return;
  }
  const auto & packet = decoded.value();
  if (!has_valid_crc(packet)) {
    RCLCPP_WARN(get_logger(), "Dropping serial packet with invalid CRC");
    return;
  }

  switch (packet.front()) {
    case PACKET_ID_LL_STATUS:
      handle_status(packet);
      break;
    case PACKET_ID_LL_IMU:
      handle_imu(packet);
      break;
    case PACKET_ID_LL_UI_EVENT:
      if (packet.size() == sizeof(LlUiEvent)) {
        const auto event = parse_packet<LlUiEvent>(packet);
        RCLCPP_INFO(
            get_logger(), "Mainboard UI event button=%u duration=%u", event.button_id,
            event.press_duration);
      }
      break;
    default:
      RCLCPP_DEBUG(get_logger(), "Ignoring unsupported mainboard packet type %u", packet.front());
      break;
  }
}

void MainboardSerialBridgeNode::handle_status(const std::vector<uint8_t> & packet)
{
  if (packet.size() != sizeof(LlStatus)) {
    RCLCPP_WARN(get_logger(), "Unexpected LL status packet size: %zu", packet.size());
    return;
  }

  const auto status = parse_packet<LlStatus>(packet);
  const bool charger_present = status.v_charge >= charger_present_voltage_threshold_;
  const bool charging_enabled = (status.status_bitmask & LL_STATUS_BIT_CHARGING) != 0;
  const bool emergency = status.emergency_bitmask != 0;
  const bool rain = (status.status_bitmask & LL_STATUS_BIT_RAIN) != 0;
  low_level_emergency_.store(emergency);
  if (!emergency) {
    release_requested_.store(false);
  }

  std_msgs::msg::Bool bool_msg;
  bool_msg.data = charger_present;
  charger_present_pub_->publish(bool_msg);

  bool_msg.data = emergency;
  emergency_pub_->publish(bool_msg);

  bool_msg.data = rain;
  rain_pub_->publish(bool_msg);

  std_msgs::msg::Float32 charge_voltage;
  charge_voltage.data = status.v_charge;
  charge_voltage_pub_->publish(charge_voltage);

  sensor_msgs::msg::BatteryState battery;
  battery.header.stamp = now();
  battery.header.frame_id = battery_frame_id_;
  battery.voltage = status.v_system;
  battery.current = status.charging_current;
  battery.percentage = static_cast<float>(status.batt_percentage) / 100.0F;
  if (!charger_present) {
    battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  } else if (status.batt_percentage >= 100) {
    battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
  } else if (!charging_enabled) {
    battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
  } else {
    battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  }
  battery.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  battery.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
  battery_pub_->publish(battery);
}

void MainboardSerialBridgeNode::handle_imu(const std::vector<uint8_t> & packet)
{
  if (packet.size() != sizeof(LlImu)) {
    RCLCPP_WARN(get_logger(), "Unexpected LL IMU packet size: %zu", packet.size());
    return;
  }

  const auto imu_in = parse_packet<LlImu>(packet);

  sensor_msgs::msg::Imu imu;
  imu.header.stamp = now();
  imu.header.frame_id = imu_frame_id_;
  imu.linear_acceleration.x = imu_in.acceleration_mss[0];
  imu.linear_acceleration.y = imu_in.acceleration_mss[1];
  imu.linear_acceleration.z = imu_in.acceleration_mss[2];
  imu.angular_velocity.x = imu_in.gyro_rads[0];
  imu.angular_velocity.y = imu_in.gyro_rads[1];
  imu.angular_velocity.z = imu_in.gyro_rads[2];
  imu.orientation_covariance[0] = -1.0;
  imu_pub_->publish(imu);
}

void MainboardSerialBridgeNode::send_heartbeat()
{
  LlHeartbeat heartbeat{};
  heartbeat.type = PACKET_ID_LL_HEARTBEAT;
  heartbeat.emergency_requested =
      static_cast<uint8_t>(emergency_requested_.load() && !low_level_emergency_.load());
  heartbeat.emergency_release_requested = static_cast<uint8_t>(release_requested_.load());
  heartbeat.crc = crc_ccitt(reinterpret_cast<const uint8_t *>(&heartbeat), sizeof(heartbeat) - 2);
  write_packet(reinterpret_cast<const uint8_t *>(&heartbeat), sizeof(heartbeat));
}

bool MainboardSerialBridgeNode::write_packet(const uint8_t * data, size_t size)
{
  const auto encoded = cobs_encode(data, size);
  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (serial_fd_ < 0) {
    return false;
  }

  size_t written_total = 0;
  while (written_total < encoded.size()) {
    const ssize_t written =
        ::write(serial_fd_, encoded.data() + written_total, encoded.size() - written_total);
    if (written < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        continue;
      }
      RCLCPP_WARN(get_logger(), "Serial write failed: %s", std::strerror(errno));
      ::close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }
    written_total += static_cast<size_t>(written);
  }
  return true;
}

}  // namespace open_mower_next::hardware_bridge
