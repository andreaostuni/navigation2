cmake_minimum_required(VERSION 3.5)
project(nav2_waypoint_follower)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(geographic_msgs REQUIRED)

nav2_package()

include_directories(
  include
)

## NORMAL WP

set(executable_name waypoint_follower)

add_executable(${executable_name}
  src/main.cpp
)

set(library_name ${executable_name}_core)

add_library(${library_name} SHARED
  src/waypoint_follower.cpp
)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  nav_msgs
  nav2_msgs
  nav2_util
  tf2_ros
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name} ${library_name})

ament_target_dependencies(${library_name}
  ${dependencies}
)
## GPS WP

set(gps_executable_name waypoint_follower_gps)

add_executable(${gps_executable_name}
  src/main_gps.cpp
)

set(gps_library_name ${gps_executable_name}_core)

add_library(${gps_library_name} SHARED
  src/waypoint_follower_gps.cpp
)

set(gps_dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  nav_msgs
  nav2_msgs
  nav2_util
  tf2_ros
  tf2_geometry_msgs
  robot_localization
)

ament_target_dependencies(${gps_executable_name}
  ${gps_dependencies}
)

target_link_libraries(${gps_executable_name} ${gps_library_name})

ament_target_dependencies(${gps_library_name}
  ${gps_dependencies}
)


# INSTALL

install(TARGETS ${library_name} ${gps_library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name} ${gps_executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()
