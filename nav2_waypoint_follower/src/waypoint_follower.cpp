// Copyright (c) 2019 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_waypoint_follower/waypoint_follower.hpp"

#include <fstream>
#include <memory>
#include <streambuf>
#include <string>
#include <utility>
#include <vector>

namespace nav2_waypoint_follower
{
  using rcl_interfaces::msg::ParameterType;
  using std::placeholders::_1;

WaypointFollower::WaypointFollower()
: nav2_util::LifecycleNode("waypoint_follower", "", false)
{
  RCLCPP_INFO(get_logger(), "Creating");

  declare_parameter("stop_on_failure", true);
  declare_parameter("loop_rate", 20);
  declare_parameter("global_frame_id", global_frame_id_);
}

WaypointFollower::~WaypointFollower()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

nav2_util::CallbackReturn
WaypointFollower::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  auto node = shared_from_this();
  stop_on_failure_ = get_parameter("stop_on_failure").as_bool();
  loop_rate_ = get_parameter("loop_rate").as_int();
  global_frame_id_ = get_parameter("global_frame_id").as_string();

  global_frame_id_ = nav2_util::strip_leading_slash(global_frame_id_);

  std::vector<std::string> new_args = rclcpp::NodeOptions().arguments();
  new_args.push_back("--ros-args");
  new_args.push_back("-r");
  new_args.push_back(std::string("__node:=") + this->get_name() + "_rclcpp_node");
  new_args.push_back("--");
  client_node_ = std::make_shared<rclcpp::Node>(
    "_", "", rclcpp::NodeOptions().arguments(new_args));

  nav_to_pose_client_ = rclcpp_action::create_client<ClientT>(
    client_node_, "navigate_to_pose");

  action_server_ = std::make_unique<ActionServer>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "follow_waypoints", std::bind(&WaypointFollower::followWaypointsCallback, this), false);

  from_ll_to_map_client_ = std::make_unique<
    nav2_util::ServiceClient<robot_localization::srv::FromLL,
    std::shared_ptr<nav2_util::LifecycleNode>>>(
    "/fromLL",
    node);

  gps_action_server_ = std::make_unique<ActionServerGPS>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "follow_gps_waypoints",
    std::bind(&WaypointFollower::followGPSWaypointsCallback, this));
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WaypointFollower::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  action_server_->activate();
  gps_action_server_->activate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WaypointFollower::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_->deactivate();
  gps_action_server_->deactivate();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WaypointFollower::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  action_server_.reset();
  gps_action_server_.reset();
  from_ll_to_map_client_.reset();
  nav_to_pose_client_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WaypointFollower::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

template<typename T>
std::vector<geometry_msgs::msg::PoseStamped> WaypointFollower::getLatestGoalPoses(
  const T & action_server)
{
  std::vector<geometry_msgs::msg::PoseStamped> poses;

  // compile time static check to decide which block of code to be built
  if constexpr (std::is_same<T, std::unique_ptr<ActionServer>>::value) {
    // If normal waypoint following callback was called, we build here
    poses = action_server->get_current_goal()->poses;
  } else {
    // If GPS waypoint following callback was called, we build here
    poses = convertGPSPosesToMapPoses(
      action_server->get_current_goal()->gps_poses);
  }
  return poses;
}

template<typename T, typename V, typename Z>
void WaypointFollower::followWaypointsHandler(
  const T & action_server,
  const V & feedback,
  const Z & result)
{
  auto goal = action_server->get_current_goal();

  std::vector<geometry_msgs::msg::PoseStamped> poses;
  poses = getLatestGoalPoses<T>(action_server);

  if (!action_server || !action_server->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server inactive. Stopping.");
    return;
  }

  RCLCPP_INFO(
    get_logger(), "Received follow waypoint request with %i waypoints.",
    static_cast<int>(poses.size()));

  if (poses.empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "Empty vector of waypoints passed to waypoint following "
      "action potentially due to conversation failure or empty request."
    );
    action_server->terminate_current(result);
    return;
  }

  rclcpp::WallRate r(loop_rate_);
  uint32_t goal_index = 0;
  bool new_goal = true;

  while (rclcpp::ok()) {
    // Check if asked to stop processing action
    if (action_server->is_cancel_requested()) {
      auto cancel_future = nav_to_pose_client_->async_cancel_all_goals();
      rclcpp::spin_until_future_complete(client_node_, cancel_future);
      // for result callback processing
      spin_some(client_node_);
      action_server->terminate_all();
      return;
    }

    // Check if asked to process another action
    if (action_server->is_preempt_requested()) {
      RCLCPP_INFO(get_logger(), "Preempting the goal pose.");
      goal = action_server->accept_pending_goal();
      goal_index = 0;
      new_goal = true;
    }

    // Check if we need to send a new goal
    if (new_goal) {
      new_goal = false;
      ClientT::Goal client_goal;
      client_goal.pose = poses[goal_index];
      client_goal.pose.header.stamp = this->now();

      auto send_goal_options = rclcpp_action::Client<ClientT>::SendGoalOptions();
      send_goal_options.result_callback =
               std::bind(&WaypointFollower::resultCallback<rclcpp_action::ClientGoalHandle<ClientT>::WrappedResult>, this, std::placeholders::_1);
      send_goal_options.goal_response_callback =
        std::bind(
        &WaypointFollower::goalResponseCallback<std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr>>, this,
        std::placeholders::_1);
      future_goal_handle_ =
        nav_to_pose_client_->async_send_goal(client_goal, send_goal_options);
      current_goal_status_ = ActionStatus::PROCESSING;
    }

    feedback->current_waypoint = goal_index;
    action_server->publish_feedback(feedback);

    if (current_goal_status_ == ActionStatus::FAILED) {
      failed_ids_.push_back(goal_index);

      if (stop_on_failure_) {
        RCLCPP_WARN(
          get_logger(), "Failed to process waypoint %i in waypoint "
          "list and stop on failure is enabled."
          " Terminating action.", goal_index);
        result->missed_waypoints = failed_ids_;
        action_server->terminate_current(result);
        failed_ids_.clear();
        return;
      } else {
        RCLCPP_INFO(
          get_logger(), "Failed to process waypoint %i,"
          " moving to next.", goal_index);
      }
    } else if (current_goal_status_ == ActionStatus::SUCCEEDED) {
      RCLCPP_INFO(
        get_logger(), "Succeeded processing waypoint %i, "
        "moving to next.", goal_index);
    }

    if (current_goal_status_ != ActionStatus::PROCESSING &&
      current_goal_status_ != ActionStatus::UNKNOWN)
    {
      // Update server state
      goal_index++;
      new_goal = true;
      if (goal_index >= poses.size()) {
        RCLCPP_INFO(
          get_logger(), "Completed all %i waypoints requested.",
          poses.size());
        result->missed_waypoints = failed_ids_;
        action_server->succeeded_current(result);
        failed_ids_.clear();
        return;
      }
    } else {
      RCLCPP_INFO_EXPRESSION(
        get_logger(),
        (static_cast<int>(now().seconds()) % 30 == 0),
        "Processing waypoint %i...", goal_index);
    }

    rclcpp::spin_some(client_node_);
    r.sleep();
  }
}

void WaypointFollower::followWaypointsCallback()
{
  auto feedback = std::make_shared<ActionT::Feedback>();
  auto result = std::make_shared<ActionT::Result>();

  followWaypointsHandler<std::unique_ptr<ActionServer>,
    ActionT::Feedback::SharedPtr,
    ActionT::Result::SharedPtr>(
    action_server_,
    feedback, result);
}

void WaypointFollower::followGPSWaypointsCallback()
{
  auto feedback = std::make_shared<ActionTGPS::Feedback>();
  auto result = std::make_shared<ActionTGPS::Result>();

  followWaypointsHandler<std::unique_ptr<ActionServerGPS>,
    ActionTGPS::Feedback::SharedPtr,
    ActionTGPS::Result::SharedPtr>(
    gps_action_server_,
    feedback, result);
}

template<typename T>
void WaypointFollower::resultCallback(
  const T & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      current_goal_status_ = ActionStatus::SUCCEEDED;
      return;
    case rclcpp_action::ResultCode::ABORTED:
      current_goal_status_ = ActionStatus::FAILED;
      return;
    case rclcpp_action::ResultCode::CANCELED:
      current_goal_status_ = ActionStatus::FAILED;
      return;
    default:
      current_goal_status_ = ActionStatus::UNKNOWN;
      return;
  }
}

template<typename T>
void WaypointFollower::goalResponseCallback(
  T future)
{
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(
      get_logger(),
      "navigate_to_pose action client failed to send goal to server.");
    current_goal_status_ = ActionStatus::FAILED;
  }
}

rcl_interfaces::msg::SetParametersResult
WaypointFollower::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  // No locking required as action server is running on same single threaded executor
  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();
    if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == "loop_rate") {
        loop_rate_ = parameter.as_int();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == "stop_on_failure") {
        stop_on_failure_ = parameter.as_bool();
      }
    }
  }
  result.successful = true;
  return result;
}

std::vector<geometry_msgs::msg::PoseStamped>
WaypointFollower::convertGPSPosesToMapPoses(
  const std::vector<geographic_msgs::msg::GeoPose> & gps_poses)
{
  RCLCPP_INFO(
    this->get_logger(), "Converting GPS waypoints to %s Frame..",
    global_frame_id_.c_str());

  std::vector<geometry_msgs::msg::PoseStamped> poses_in_map_frame_vector;
  int waypoint_index = 0;
  for (auto && curr_geopose : gps_poses) {
    auto request = std::make_shared<robot_localization::srv::FromLL::Request>();
    auto response = std::make_shared<robot_localization::srv::FromLL::Response>();
    request->ll_point.latitude = curr_geopose.position.latitude;
    request->ll_point.longitude = curr_geopose.position.longitude;
    request->ll_point.altitude = curr_geopose.position.altitude;

    from_ll_to_map_client_->wait_for_service((std::chrono::seconds(1)));
    if (!from_ll_to_map_client_->invoke(request, response)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "fromLL service of robot_localization could not convert %i th GPS waypoint to"
        "%s frame, going to skip this point!"
        "Make sure you have run navsat_transform_node of robot_localization",
        waypoint_index, global_frame_id_.c_str());
      if (stop_on_failure_) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Conversion of %i th GPS waypoint to"
          "%s frame failed and stop_on_failure is set to true"
          "Not going to execute any of waypoints, exiting with failure!",
          waypoint_index, global_frame_id_.c_str());
        return std::vector<geometry_msgs::msg::PoseStamped>();
      }
      continue;
    } else {
      geometry_msgs::msg::PoseStamped curr_pose_map_frame;
      curr_pose_map_frame.header.frame_id = global_frame_id_;
      curr_pose_map_frame.header.stamp = this->now();
      curr_pose_map_frame.pose.position = response->map_point;
      curr_pose_map_frame.pose.orientation = curr_geopose.orientation;
      poses_in_map_frame_vector.push_back(curr_pose_map_frame);
    }
    waypoint_index++;
  }
  RCLCPP_INFO(
    this->get_logger(),
    "Converted all %i GPS waypoint to %s frame",
    static_cast<int>(poses_in_map_frame_vector.size()), global_frame_id_.c_str());
  return poses_in_map_frame_vector;
}

}  // namespace nav2_waypoint_follower

