#ifndef NAV2_WAYPOINT_FOLLOWER__WAYPOINT_FOLLOWER_GPS_HPP_
#define NAV2_WAYPOINT_FOLLOWER__WAYPOINT_FOLLOWER_GPS_HPP_

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
  SUCCEEDED = 3
};

/**
 * @class nav2_waypoint_follower::WaypointFollowerGPS
 * @brief An action server that uses behavior tree for navigating a robot to its
 * goal position.
 */
class WaypointFollowerGPS : public nav2_util::LifecycleNode
{
public:
  using ActionT = nav2_msgs::action::FollowGPSWaypoints;
  using ClientT = nav2_msgs::action::NavigateToPose;
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;
  using ActionClient = rclcpp_action::Client<ClientT>;

  /**
   * @brief A constructor for nav2_waypoint_follower::WaypointFollower class
   */
  WaypointFollowerGPS();
  /**
   * @brief A destructor for nav2_waypoint_follower::WaypointFollower class
   */
  ~WaypointFollowerGPS();

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
   * @brief Action server callbacks
   */
  void followGPSWaypoints();

  /**
   * @brief Action client result callback
   * @param result Result of action server updated asynchronously
   */
  void resultCallback(const rclcpp_action::ClientGoalHandle<ClientT>::WrappedResult& result);

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

  // Our action server
  std::unique_ptr<ActionServer> action_server_;
  ActionClient::SharedPtr nav_to_pose_client_;
  rclcpp::Node::SharedPtr client_node_;

  std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr> future_goal_handle_;
  bool stop_on_failure_;
  ActionStatus current_goal_status_;
  std::string global_frame_id_{ "map" };
  int loop_rate_;
  std::vector<int> failed_ids_;
  std::unique_ptr<nav2_util::ServiceClient<robot_localization::srv::FromLL>> from_ll_to_map_client_;
};

}  // namespace nav2_waypoint_follower

#endif /* NAV2_WAYPOINT_FOLLOWER__WAYPOINT_FOLLOWER_GPS_HPP_ */
