#ifndef NAVIGATORS_GPS_NAVIGATOR_HPP_
#define NAVIGATORS_GPS_NAVIGATOR_HPP_

#include <string>
#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geographic_msgs/msg/geo_pose.hpp"
#include "nav2_core/behavior_tree_navigator.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/service_client.hpp"
#include "robot_localization/srv/from_ll.hpp"

namespace nav2_bt_navigator
{

/**
 * @class GPSNavigator
 * @brief GPSNavigator interface that acts as a base class for all BT-based Navigator action's plugins
 */
template<class ActionT>
class GPSNavigator: virtual public nav2_core::BehaviorTreeNavigator<ActionT>
{
public:

  /**
   * @brief A GPSNavigator constructor
   */
  GPSNavigator(): nav2_core::BehaviorTreeNavigator<ActionT>() {
    from_ll_to_map_client_ = nullptr;
  }

  /**
   * @brief Virtual destructor
   */
  virtual ~GPSNavigator() = default;

  /**
   * @brief A configure state transition to configure navigator's state
   * @param node Weakptr to the lifecycle node
   * @param odom_smoother Object to get current smoothed robot's speed
   */
  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override 
    {
      auto node = parent_node.lock();
      if (!node->has_parameter("global_frame")) {
        node->declare_parameter("global_frame", std::string("map"));
      }

      global_frame_id_ = node->get_parameter("global_frame").as_string();
      RCLCPP_INFO(node->get_logger(), "GPSNavigator::odom_smoother_ = %s", odom_smoother->getTwistStamped().header.frame_id.c_str());

      from_ll_to_map_client_ = std::make_unique<nav2_util::ServiceClient
        <robot_localization::srv::FromLL,std::shared_ptr<rclcpp_lifecycle::LifecycleNode>>>("/fromLL", node );
      return true;
    }

  /**
   * @brief A cleanup state transition to remove memory allocated
   */
  bool cleanup() override
  {
    from_ll_to_map_client_.reset();
    return true;
  }

protected:

  /**
   * @brief Method to convert the gps_poses to actual poses in the current map
   */
  std::vector<geometry_msgs::msg::PoseStamped>
    convertGPSPosesToMapPoses(
    const std::vector<geographic_msgs::msg::GeoPose> & gps_poses)
  {
    RCLCPP_INFO(
      this->logger_, "Converting GPS waypoints to %s Frame..",
      global_frame_id_.c_str());

    std::vector<geometry_msgs::msg::PoseStamped> poses_in_map_frame_vector;
    geometry_msgs::msg::PoseStamped curr_pose_map_frame;
    for (auto && curr_geopose : gps_poses) {
      curr_pose_map_frame = convertGPSPoseToMapPose(curr_geopose);
      if (!checkPoseValidity(curr_pose_map_frame)) {
        RCLCPP_ERROR(this->logger_,
          "Conversion of GPS waypoint to %s frame failed",global_frame_id_.c_str());
        return std::vector<geometry_msgs::msg::PoseStamped>();
      }
      poses_in_map_frame_vector.push_back(curr_pose_map_frame);
    }
    RCLCPP_INFO(
      this->logger_,
      "Converted all %i GPS poses to %s frame",
      static_cast<int>(poses_in_map_frame_vector.size()), global_frame_id_.c_str());
    return poses_in_map_frame_vector;
  }

  /**
   * @brief Method to convert the gps_poses to actual poses in the current map
   */
  geometry_msgs::msg::PoseStamped
    convertGPSPoseToMapPose(
    const geographic_msgs::msg::GeoPose & gps_pose)
  {
    auto request = std::make_shared<robot_localization::srv::FromLL::Request>();
    auto response = std::make_shared<robot_localization::srv::FromLL::Response>();
    request->ll_point.latitude = gps_pose.position.latitude;
    request->ll_point.longitude = gps_pose.position.longitude;
    request->ll_point.altitude = gps_pose.position.altitude;

    from_ll_to_map_client_->wait_for_service((std::chrono::seconds(1)));
    if (!from_ll_to_map_client_->invoke(request, response)) {
      RCLCPP_ERROR(this->logger_,
        "fromLL service of robot_localization could not convert the pose to"
        "%s frame, going to skip this point!"
        "Make sure you have run navsat_transform_node of robot_localization",
         global_frame_id_.c_str());
      return geometry_msgs::msg::PoseStamped();
    } 
    else {      
      geometry_msgs::msg::PoseStamped pose_map_frame;
      pose_map_frame.header.frame_id = global_frame_id_;
      pose_map_frame.header.stamp = this->clock_->now();
      pose_map_frame.pose.position = response->map_point;
      pose_map_frame.pose.orientation = gps_pose.orientation;
      return pose_map_frame;
    }
  }

  bool checkPoseValidity(const geometry_msgs::msg::PoseStamped & pose)
  {
    if (pose.header.frame_id != global_frame_id_) {
      RCLCPP_ERROR(this->logger_,
        "GPS pose is not in %s frame, going to skip this point!"
        "Make sure you have run navsat_transform_node of robot_localization",
         global_frame_id_.c_str());
      return false;
    }
    return true;
  }
  std::string global_frame_id_;
  std::unique_ptr<nav2_util::ServiceClient<robot_localization::srv::FromLL,std::shared_ptr<rclcpp_lifecycle::LifecycleNode>>> from_ll_to_map_client_;
};

}  // namespace nav2_bt_navigator


#endif /* NAVIGATORS_GPS_NAVIGATOR_HPP_ */
