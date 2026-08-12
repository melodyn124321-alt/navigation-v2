cmake_minimum_required(VERSION 3.8)
project(pcd_ndt_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io registration)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(pcd_ndt_localizer src/pcd_ndt_localizer.cpp)
target_include_directories(pcd_ndt_localizer PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pcd_ndt_localizer ${PCL_LIBRARIES})
ament_target_dependencies(pcd_ndt_localizer
  geometry_msgs
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  tf2
  tf2_eigen
  tf2_ros
  visualization_msgs
)

add_executable(pcd_occupancy_grid_publisher src/pcd_occupancy_grid_publisher.cpp)
target_include_directories(pcd_occupancy_grid_publisher PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pcd_occupancy_grid_publisher ${PCL_LIBRARIES})
ament_target_dependencies(pcd_occupancy_grid_publisher
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
)

install(TARGETS
  pcd_ndt_localizer
  pcd_occupancy_grid_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
