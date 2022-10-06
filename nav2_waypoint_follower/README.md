# Nav2 Waypoint Follower

The navigation2 waypoint follower is an example application of how to use the navigation action to complete some sort of orchestrated task. In this example, that task is to take a given set of waypoints and navigate to a set of positions in the order provided in the action request. The last waypoint in the waypoint array is the final position.

The package exposes the `FollowWaypoints` action server of type `nav2_msgs/FollowWaypoints`. It is given an array of waypoints to visit, gives feedback about the current index of waypoint it is processing, and returns a list of waypoints it was unable to complete.

There is a parameterization `stop_on_failure` whether to stop processing the waypoint following action on a single waypoint failure. When false, it will continue onto the next waypoint when the current waypoint fails. The action will exist when either all the waypoint navigation tasks have terminated or when `stop_on_failure`, a single waypoint as failed.

## Nav2 GPS Waypoint Follower

`nav2_waypoint_follower` provides an action server named `FollowGPSWaypoints` which accepts GPS waypoint following requests by using tools provided by `robot_localization` and `nav2_waypoint_follower` itself.

`robot_localization`'s `navsat_transform_node` provides a service `fromLL`, which is used to convert pure GPS coordinates(longitude, latitude, alitude)
to cartesian coordinates in map frame(x,y), then the existent action named `FollowWaypoints` from `nav2_waypoint_follower` is used to get robot go through each converted waypoints. 
The action msg definition for GPS waypoint following can be found [here](../nav2_msgs/action/FollowGPSWaypoints.action).

In a common use case, an client node can read a set of GPS waypoints from a YAML file an create a client to action server named as `FollowGPSWaypoints`.  
For instance,

```cpp
using ClientT = nav2_msgs::action::FollowGPSWaypoints;
rclcpp_action::Client<ClientT>::SharedPtr gps_waypoint_follower_action_client_;
gps_waypoint_follower_action_client_ = rclcpp_action::create_client<ClientT>(this, "follow_gps_waypoints");
```

All other functionalities provided by `nav2_waypoint_follower` such as WaypointTaskExecutors are usable and can be configured in WaypointTaskExecutor.