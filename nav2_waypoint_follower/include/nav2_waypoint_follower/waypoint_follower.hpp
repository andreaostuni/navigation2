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

#ifndef NAV2_WAYPOINT_FOLLOWER__WAYPOINT_FOLLOWER_HPP_
#define NAV2_WAYPOINT_FOLLOWER__WAYPOINT_FOLLOWER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp_action/rclcpp_action.hpp"
#include "geographic_msgs/msg/geo_pose.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_util/string_utils.hpp"
#include "nav2_msgs/action/follow_gps_waypoints.hpp"
#include "nav2_util/service_client.hpp"
#include "robot_localization/srv/from_ll.hpp"

namespace nav2_waypoint_follower
{
enum class ActionStatus
{
  UNKNOWN = 0,
  PROCESSING = 1,
  FAILED = 2,
  SUCCEEDED = 3,
  PREEMPTED = 4
};

/**
 * @class nav2_waypoint_follower::WaypointFollower
 * @brief An action server that uses behavior tree for navigating a robot to its
 * goal position.
 */
class WaypointFollower : public nav2_util::LifecycleNode
{
public:
  using ActionT = nav2_msgs::action::FollowWaypoints;
  using ClientT = nav2_msgs::action::NavigateToPose;
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;
  using ActionClient = rclcpp_action::Client<ClientT>;

  // Shorten the types for GPS waypoint following
  using ActionTGPS = nav2_msgs::action::FollowGPSWaypoints;
  using ActionServerGPS = nav2_util::SimpleActionServer<ActionTGPS>;

  /**
   * @brief A constructor for nav2_waypoint_follower::WaypointFollower class
   */
  WaypointFollower();
  /**
   * @brief A destructor for nav2_waypoint_follower::WaypointFollower class
   */
  ~WaypointFollower();

protected:
  /**
   * @brief Configures member variables
   *
   * Initializes action server for "FollowWaypoints"
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  /**
   * @brief Activates action server
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  /**
   * @brief Deactivates action server
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  /**
   * @brief Resets member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief Templated function to perform internal logic behind waypoint following,
   *        Both GPS and non GPS waypoint following callbacks makes use of this function when a client asked to do so.
   *        Callbacks fills in appropriate types for the tempelated types, see followWaypointCallback funtions for
   * details.
   *
   * @tparam T action_server
   * @tparam V feedback
   * @tparam Z result
   * @param action_server
   * @param poses
   * @param feedback
   * @param result
   */
  template <typename T, typename V, typename Z>
  void followWaypointsHandler(const T& action_server, const V& feedback, const Z& result);

  /**
   * @brief Action server callbacks
   */
  void followWaypointsCallback();

  /**
   * @brief send robot through each of GPS
   *        point , which are converted to map frame first then using a client to
   *        `FollowWaypoints` action.
   *
   * @param waypoints, acquired from action client
   */
  void followGPSWaypointsCallback();

  /**
   * @brief Action client result callback
   * @param result Result of action server updated asynchronously
   */
  template <typename T>
  void resultCallback(const T& result);
  /**
   * @brief Action client goal response callback
   * @param future Shared future to goalhandle
   */
  void goalResponseCallback(std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr> future);

  /**
   * @brief given some gps_poses, converts them to map frame using robot_localization's service `fromLL`.
   *        Constructs a vector of stamped poses in map frame and returns them.
   *
   * @param gps_poses, from the action server
   * @return std::vector<geometry_msgs::msg::PoseStamped>
   */
  std::vector<geometry_msgs::msg::PoseStamped>
  convertGPSPosesToMapPoses(const std::vector<geographic_msgs::msg::GeoPose>& gps_poses);

  /**
   * @brief get the latest poses on the action server goal. If they are GPS poses,
   * convert them to the global cartesian frame using /fromLL robot localization
   * server
   *
   * @param action_server, to which the goal was sent
   * @return std::vector<geometry_msgs::msg::PoseStamped>
   */
  template <typename T>
  std::vector<geometry_msgs::msg::PoseStamped> getLatestGoalPoses(const T& action_server);

  // Our action server
  std::unique_ptr<ActionServer> action_server_;
  ActionClient::SharedPtr nav_to_pose_client_;
  rclcpp::Node::SharedPtr client_node_;
  std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr> future_goal_handle_;
  bool stop_on_failure_;
  ActionStatus current_goal_status_;
  int loop_rate_;
  std::string global_frame_id_{ "map" };
  std::vector<int> failed_ids_;

  // Our action server for GPS waypoint following
  std::unique_ptr<ActionServerGPS> gps_action_server_;
  std::unique_ptr<nav2_util::ServiceClient<robot_localization::srv::FromLL>> from_ll_to_map_client_;
};

}  // namespace nav2_waypoint_follower

#endif  // NAV2_WAYPOINT_FOLLOWER__WAYPOINT_FOLLOWER_HPP_
