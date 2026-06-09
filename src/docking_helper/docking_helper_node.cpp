#include "docking_helper/docking_helper_node.hpp"
#include "docking_helper_node.hpp"
#include <atomic>
#include <algorithm>
#include <cmath>
#include <functional>
#include <future>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::placeholders;

open_mower_next::docking_helper::DockingHelperNode::DockingHelperNode(const rclcpp::NodeOptions& options)
  : Node("docking_helper", options)
{
  map_sub_ = create_subscription<open_mower_next::msg::Map>(
      "/mowing_map", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal),
      std::bind(&DockingHelperNode::mapCallback, this, std::placeholders::_1));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

  find_nearest_docking_station_service_ = create_service<open_mower_next::srv::FindNearestDockingStation>(
      "/find_nearest_docking_station", std::bind(&DockingHelperNode::findNearestDockingStationService, this,
                                                 std::placeholders::_1, std::placeholders::_2));

  dock_client_ = rclcpp_action::create_client<nav2_msgs::action::DockRobot>(this, "/dock_robot");
  navigate_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "/navigate_to_pose");
  docking_cmd_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_joy", 10);
  charger_present_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/power/charger_present", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
        charger_present_.store(msg->data);
      });

  docking_linear_velocity_ = declare_parameter<double>("docking_linear_velocity", 0.08);
  docking_angular_gain_ = declare_parameter<double>("docking_angular_gain", 1.5);
  docking_max_angular_velocity_ = declare_parameter<double>("docking_max_angular_velocity", 0.25);
  docking_yaw_tolerance_ = declare_parameter<double>("docking_yaw_tolerance", 0.08);
  docking_drive_timeout_sec_ = declare_parameter<double>("docking_drive_timeout_sec", 25.0);

  dock_robot_nearest_server_ = rclcpp_action::create_server<DockRobotNearestAction>(
      this, "dock_robot_nearest", std::bind(&DockingHelperNode::handleDockRobotNearestGoal, this, _1, _2),
      std::bind(&DockingHelperNode::handleDockRobotNearestCancel, this, _1),
      std::bind(&DockingHelperNode::handleDockRobotNearestAccepted, this, _1));

  dock_robot_to_server_ = rclcpp_action::create_server<DockRobotToAction>(
      this, "dock_robot_to", std::bind(&DockingHelperNode::handleDockRobotToGoal, this, _1, _2),
      std::bind(&DockingHelperNode::handleDockRobotToCancel, this, _1),
      std::bind(&DockingHelperNode::handleDockRobotToAccepted, this, _1));
}

open_mower_next::docking_helper::DockingHelperNode::~DockingHelperNode()
{
  // No specific cleanup required
}

void open_mower_next::docking_helper::DockingHelperNode::mapCallback(const open_mower_next::msg::Map::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Received map with %zu docking stations", msg->docking_stations.size());

  {
    std::lock_guard<std::mutex> lock(docking_stations_mutex_);
    docking_stations_.clear();

    for (const auto& docking_station : msg->docking_stations)
    {
      RCLCPP_DEBUG(get_logger(), "Docking station: %s", docking_station.name.c_str());

      docking_stations_.push_back(docking_station);
    }
  }
}

std::shared_ptr<open_mower_next::msg::DockingStation>
open_mower_next::docking_helper::DockingHelperNode::findNearestDockingStation()
{
  std::lock_guard<std::mutex> lock(docking_stations_mutex_);

  if (docking_stations_.empty())
  {
    RCLCPP_ERROR(get_logger(), "No docking stations available");
    return nullptr;
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  geometry_msgs::msg::TransformStamped transform;

  try
  {
    transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

    robot_pose.header.frame_id = "map";
    robot_pose.header.stamp = this->now();
    robot_pose.pose.position.x = transform.transform.translation.x;
    robot_pose.pose.position.y = transform.transform.translation.y;
    robot_pose.pose.position.z = transform.transform.translation.z;
    robot_pose.pose.orientation = transform.transform.rotation;
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_ERROR(get_logger(), "Could not transform pose: %s", ex.what());
    return nullptr;
  }

  double min_distance = std::numeric_limits<double>::max();
  open_mower_next::msg::DockingStation nearest_station;

  for (const auto& station : docking_stations_)
  {
    double dx = station.pose.pose.position.x - robot_pose.pose.position.x;
    double dy = station.pose.pose.position.y - robot_pose.pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance < min_distance)
    {
      min_distance = distance;
      nearest_station = station;
    }
  }

  RCLCPP_INFO(get_logger(), "Found nearest docking station at distance: %f meters", min_distance);
  return std::make_shared<open_mower_next::msg::DockingStation>(nearest_station);
}

void open_mower_next::docking_helper::DockingHelperNode::findNearestDockingStationService(
    const std::shared_ptr<open_mower_next::srv::FindNearestDockingStation::Request> request,
    std::shared_ptr<open_mower_next::srv::FindNearestDockingStation::Response> response)
{
  try
  {
    auto nearest_station = findNearestDockingStation();

    if (!nearest_station)
    {
      response->code = open_mower_next::srv::FindNearestDockingStation::Response::CODE_NOT_FOUND;
      RCLCPP_ERROR(get_logger(), "Failed to find nearest docking station");
      return;
    }

    response->docking_station = *nearest_station;
    response->code = open_mower_next::srv::FindNearestDockingStation::Response::CODE_SUCCESS;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "Exception occured while finding nearest docking station: %s", e.what());
    response->code = open_mower_next::srv::FindNearestDockingStation::Response::CODE_UNKNOWN_ERROR;
    return;
  }
}

std::shared_ptr<geometry_msgs::msg::PoseStamped> open_mower_next::docking_helper::DockingHelperNode::dockPose(
    const std::shared_ptr<open_mower_next::msg::DockingStation>& station)
{
  if (!station)
  {
    RCLCPP_ERROR(get_logger(), "Cannot transform null docking station");
    return nullptr;
  }

  auto pose_stamped = std::make_shared<geometry_msgs::msg::PoseStamped>();
  pose_stamped->header = station->pose.header;
  pose_stamped->pose = station->pose.pose;
  return pose_stamped;
}

bool open_mower_next::docking_helper::DockingHelperNode::navigateToApproachPose(
    const open_mower_next::msg::DockingStation& docking_station)
{
  if (docking_station.approach_pose.header.frame_id.empty())
  {
    RCLCPP_ERROR(get_logger(), "Docking station %s has no approach pose", docking_station.name.c_str());
    return false;
  }

  if (!navigate_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(get_logger(), "NavigateToPose action server not available");
    return false;
  }

  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose = docking_station.approach_pose;
  goal.pose.header.stamp = this->now();

  RCLCPP_INFO(get_logger(), "Navigating to docking approach pose for station: %s", docking_station.name.c_str());

  auto result_promise = std::make_shared<std::promise<bool>>();
  auto result_future = result_promise->get_future();
  auto result_set = std::make_shared<std::atomic<bool>>(false);

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback = [this, result_set, result_promise](const auto& goal_handle) {
    if (!goal_handle)
    {
      RCLCPP_ERROR(get_logger(), "NavigateToPose goal to docking approach was rejected");
      if (!result_set->exchange(true))
      {
        result_promise->set_value(false);
      }
    }
  };
  send_goal_options.result_callback = [this, result_set, result_promise](const auto& nav_result) {
    const bool success = nav_result.code == rclcpp_action::ResultCode::SUCCEEDED;
    if (success)
    {
      RCLCPP_INFO(get_logger(), "Reached docking approach pose");
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to reach docking approach pose");
    }
    if (!result_set->exchange(true))
    {
      result_promise->set_value(success);
    }
  };

  navigate_client_->async_send_goal(goal, send_goal_options);

  if (result_future.wait_for(std::chrono::minutes(5)) != std::future_status::ready)
  {
    RCLCPP_ERROR(get_logger(), "Timed out while navigating to docking approach pose");
    return false;
  }

  return result_future.get();
}

void open_mower_next::docking_helper::DockingHelperNode::publishDockingVelocity(double linear_x, double angular_z)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = this->now();
  cmd.header.frame_id = "base_link";
  cmd.twist.linear.x = linear_x;
  cmd.twist.angular.z = angular_z;
  docking_cmd_vel_pub_->publish(cmd);
}

void open_mower_next::docking_helper::DockingHelperNode::stopDockingVelocity()
{
  for (int i = 0; i < 5; ++i)
  {
    publishDockingVelocity(0.0, 0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

bool open_mower_next::docking_helper::DockingHelperNode::driveDockingVectorUntilCharging(
    const open_mower_next::msg::DockingStation& docking_station)
{
  tf2::Quaternion q;
  tf2::fromMsg(docking_station.pose.pose.orientation, q);
  double roll, pitch, target_yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, target_yaw);

  RCLCPP_INFO(
      get_logger(), "Driving docking vector until charging is detected: target yaw %.3f rad", target_yaw);

  const auto start_time = this->now();
  rclcpp::Rate rate(20.0);
  bool edge_reached = false;

  while (rclcpp::ok())
  {
    if (charger_present_.load())
    {
      stopDockingVelocity();
      RCLCPP_INFO(get_logger(), "Charging detected during docking vector drive");
      return true;
    }

    const auto elapsed = (this->now() - start_time).seconds();
    if (elapsed > docking_drive_timeout_sec_)
    {
      stopDockingVelocity();
      RCLCPP_ERROR(get_logger(), "Timed out while driving docking vector");
      return false;
    }

    geometry_msgs::msg::TransformStamped robot_transform;
    try
    {
      robot_transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    }
    catch (const tf2::TransformException& ex)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000, "Could not transform robot pose for docking drive: %s", ex.what());
      publishDockingVelocity(0.0, 0.0);
      rate.sleep();
      continue;
    }

    tf2::Quaternion robot_q;
    tf2::fromMsg(robot_transform.transform.rotation, robot_q);
    double robot_roll, robot_pitch, robot_yaw;
    tf2::Matrix3x3(robot_q).getRPY(robot_roll, robot_pitch, robot_yaw);

    double yaw_error = target_yaw - robot_yaw;
    while (yaw_error > M_PI)
      yaw_error -= 2.0 * M_PI;
    while (yaw_error < -M_PI)
      yaw_error += 2.0 * M_PI;

    const double angular_z =
        std::clamp(docking_angular_gain_ * yaw_error, -docking_max_angular_velocity_, docking_max_angular_velocity_);

    double linear_x = 0.0;
    if (std::abs(yaw_error) < docking_yaw_tolerance_)
    {
      linear_x = docking_linear_velocity_;
    }

    const double dx = robot_transform.transform.translation.x - docking_station.pose.pose.position.x;
    const double dy = robot_transform.transform.translation.y - docking_station.pose.pose.position.y;
    const double along_edge = dx * std::cos(target_yaw) + dy * std::sin(target_yaw);
    if (!edge_reached && along_edge >= 0.0)
    {
      edge_reached = true;
      RCLCPP_INFO(get_logger(), "Docking station edge reached, continuing until charging is detected");
    }

    publishDockingVelocity(linear_x, angular_z);
    rate.sleep();
  }

  stopDockingVelocity();
  return false;
}

rclcpp_action::GoalResponse open_mower_next::docking_helper::DockingHelperNode::handleDockRobotNearestGoal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const DockRobotNearestAction::Goal> goal)
{
  (void)uuid;
  (void)goal;
  RCLCPP_INFO(get_logger(), "Received request to dock to nearest docking station");

  {
    std::lock_guard<std::mutex> lock(docking_stations_mutex_);
    if (docking_stations_.empty())
    {
      RCLCPP_ERROR(get_logger(), "No docking stations available");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse open_mower_next::docking_helper::DockingHelperNode::handleDockRobotNearestCancel(
    const std::shared_ptr<DockRobotNearestGoalHandle> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Received request to cancel docking to nearest station");
  return rclcpp_action::CancelResponse::ACCEPT;
}

template <typename ActionT, typename GoalHandleT>
void open_mower_next::docking_helper::DockingHelperNode::executeDockingAction(
    const std::shared_ptr<GoalHandleT>& goal_handle,
    const std::shared_ptr<open_mower_next::msg::DockingStation>& docking_station)
{
  auto feedback = std::make_shared<typename ActionT::Feedback>();
  auto result = std::make_shared<typename ActionT::Result>();

  feedback->status = ActionT::Feedback::STATUS_NONE;
  feedback->num_retries = 0;
  feedback->docking_time.sec = 0;
  feedback->docking_time.nanosec = 0;

  if (!docking_station)
  {
    result->code = ActionT::Result::CODE_DOCK_NOT_IN_DB;
    result->message = "No docking station available";
    result->num_retries = 0;
    goal_handle->abort(result);
    return;
  }

  feedback->chosen_docking_station = *docking_station;
  result->chosen_docking_station = *docking_station;

  feedback->status = ActionT::Feedback::STATUS_NAV_TO_STAGING_POSE;
  feedback->message = "Starting docking to: " + docking_station->name;
  goal_handle->publish_feedback(feedback);

  auto start_time = this->now();

  std::shared_ptr<uint16_t> current_status = std::make_shared<uint16_t>(ActionT::Feedback::STATUS_NONE);
  std::shared_ptr<uint16_t> current_retries = std::make_shared<uint16_t>(0);
  std::atomic<bool> docking_active(true);

  if (!navigateToApproachPose(*docking_station))
  {
    result->code = ActionT::Result::CODE_FAILED_TO_STAGE;
    result->message = "Failed to reach docking approach pose";
    result->num_retries = 0;
    goal_handle->abort(result);
    return;
  }

  feedback->status = ActionT::Feedback::STATUS_CONTROLLING;
  feedback->message = "Approach pose reached, starting controlled docking";
  goal_handle->publish_feedback(feedback);

  if (driveDockingVectorUntilCharging(*docking_station))
  {
    docking_active = false;
    result->code = ActionT::Result::CODE_SUCCESS;
    result->message = "Docking completed successfully";
    result->num_retries = 0;
    goal_handle->succeed(result);
  }
  else
  {
    docking_active = false;
    result->code = ActionT::Result::CODE_FAILED_TO_CHARGE;
    result->message = "Failed to detect charging while driving docking vector";
    result->num_retries = 0;
    goal_handle->abort(result);
  }

  std::string status_messages[] = { "No activity",         "Navigating to staging pose", "Initial perception of dock",
                                    "Controlling to dock", "Waiting for charge",         "Retrying docking" };

  uint16_t last_status = 99;  // Invalid value to ensure first update is sent
  uint16_t last_retries = 0;

  while (docking_active && rclcpp::ok())
  {
    auto current_time = this->now();
    auto elapsed = current_time - start_time;
    feedback->docking_time.sec = elapsed.seconds();
    feedback->docking_time.nanosec = elapsed.nanoseconds() % 1000000000;

    uint16_t status = *current_status;
    uint16_t retries = *current_retries;

    if (status != last_status || retries != last_retries)
    {
      feedback->status = status;
      feedback->num_retries = retries;

      if (status < sizeof(status_messages) / sizeof(status_messages[0]))
      {
        feedback->message = status_messages[status];
        if (retries > 0)
        {
          feedback->message += " (retry " + std::to_string(retries) + ")";
        }
      }
      else
      {
        feedback->message = "Unknown status: " + std::to_string(status);
      }

      last_status = status;
      last_retries = retries;

      goal_handle->publish_feedback(feedback);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void open_mower_next::docking_helper::DockingHelperNode::handleDockRobotNearestAccepted(
    const std::shared_ptr<DockRobotNearestGoalHandle> goal_handle)
{
  std::thread{ [this, goal_handle]() {
    auto nearest_station = findNearestDockingStation();
    executeDockingAction<DockRobotNearestAction>(goal_handle, nearest_station);
  } }.detach();
}

std::shared_ptr<open_mower_next::msg::DockingStation>
open_mower_next::docking_helper::DockingHelperNode::findDockingStationById(const std::string& id)
{
  std::lock_guard<std::mutex> lock(docking_stations_mutex_);

  for (const auto& station : docking_stations_)
  {
    if (station.id == id)
    {
      return std::make_shared<open_mower_next::msg::DockingStation>(station);
    }
  }
  return nullptr;
}

rclcpp_action::GoalResponse open_mower_next::docking_helper::DockingHelperNode::handleDockRobotToGoal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const DockRobotToAction::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(get_logger(), "Received request to dock to station ID: %s", goal->dock_id.c_str());

  {
    std::lock_guard<std::mutex> lock(docking_stations_mutex_);
    if (docking_stations_.empty())
    {
      RCLCPP_ERROR(get_logger(), "No docking stations available");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  auto docking_station = findDockingStationById(goal->dock_id);
  if (!docking_station)
  {
    RCLCPP_ERROR(get_logger(), "Docking station with ID %s not found", goal->dock_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse open_mower_next::docking_helper::DockingHelperNode::handleDockRobotToCancel(
    const std::shared_ptr<DockRobotToGoalHandle> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Received request to cancel docking to station ID: %s",
              goal_handle->get_goal()->dock_id.c_str());
  return rclcpp_action::CancelResponse::ACCEPT;
}

void open_mower_next::docking_helper::DockingHelperNode::handleDockRobotToAccepted(
    const std::shared_ptr<DockRobotToGoalHandle> goal_handle)
{
  std::thread{ [this, goal_handle]() {
    auto goal = goal_handle->get_goal();
    auto docking_station = findDockingStationById(goal->dock_id);
    executeDockingAction<DockRobotToAction>(goal_handle, docking_station);
  } }.detach();
}
