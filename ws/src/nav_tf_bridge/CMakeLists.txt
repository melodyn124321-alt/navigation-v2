cmake_minimum_required(VERSION 3.8)
project(nav_tf_bridge)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(map_odom_tf_bridge src/map_odom_tf_bridge.cpp)
ament_target_dependencies(
  map_odom_tf_bridge
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
)

install(TARGETS map_odom_tf_bridge DESTINATION lib/${PROJECT_NAME})
ament_package()
