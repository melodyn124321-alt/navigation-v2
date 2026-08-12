#!/usr/bin/env bash
set -euo pipefail

kill_ros_wrappers() {
  local pattern="$1"
  local pids
  pids="$(ps -eo pid=,args= | awk -v pat="$pattern" '$0 ~ pat && $0 !~ /awk -v pat/ {print $1}')"
  if [ -n "${pids}" ]; then
    # shellcheck disable=SC2086
    kill ${pids} 2>/dev/null || true
  fi
}

force_kill_matches() {
  ps -eo pid=,comm=,args= | awk '
    /livox_ros_driver2_node|fastlio_mapping|pcd_ndt_localizer|nav2_bringup|map_server|planner_server|controller_server|smoother_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|velocity_smooth|collision_monitor|lifecycle_manag|lifecycle_manager|component_container|ranger_base_node|map_camera_init_bridge.py|map_odom_tf_bridge.py|map_odom_tf_bridge|nav_tf_direct_relay.py|odom_tf_republisher.py|nav_static_tf_bootstrap.py|nav_identity_tf_publisher.py|relocalization_tf_republisher.py|safe_cmd_vel_bridge.py|nav_goal_marker_from_plan.py|nav_obstacle_block_alarm.py|aligned_nav_goal_adapter.py|ndt_goal_controller.py|nav_motion_safety_gate.py|nav_obstacle_cloud_republisher.py|cloud_callback_probe|cloud_geometry_inspector|static_transform_publisher.*nav_lidar|ros2 bag play|ros2 bag record|rviz2|apport/ &&
    $0 !~ /awk/ &&
    $0 !~ /seeed_stop_ros_stack/ &&
    $0 !~ /seeed_start_manual_002_nav_safe/ {print $1}
  ' | xargs -r kill -9 2>/dev/null || true
}

echo "Stopping ROS2 wrappers and nodes on this host..."
kill_ros_wrappers "ros2 launch fast_lio"
kill_ros_wrappers "ros2 launch livox_ros_driver2"
kill_ros_wrappers "ros2 launch pcd_ndt_localization"
kill_ros_wrappers "ros2 launch nav2_bringup"
kill_ros_wrappers "seeed_activate_nav2_after_initialpose.sh"
kill_ros_wrappers "ros2 run nav2_map_server map_server"
kill_ros_wrappers "ros2 run nav2_planner planner_server"
kill_ros_wrappers "ros2 run nav2_controller controller_server"
kill_ros_wrappers "ros2 run nav2_smoother smoother_server"
kill_ros_wrappers "ros2 run nav2_behaviors behavior_server"
kill_ros_wrappers "ros2 run nav2_bt_navigator bt_navigator"
kill_ros_wrappers "ros2 run nav2_waypoint_follower waypoint_follower"
kill_ros_wrappers "ros2 run nav2_velocity_smoother velocity_smoother"
kill_ros_wrappers "ros2 run nav2_collision_monitor collision_monitor"
kill_ros_wrappers "ros2 run nav2_lifecycle_manager lifecycle_manager"
kill_ros_wrappers "ros2 launch ranger_base"
kill_ros_wrappers "map_camera_init_bridge.py"
kill_ros_wrappers "map_odom_tf_bridge.py"
kill_ros_wrappers "nav_tf_bridge map_odom_tf_bridge"
kill_ros_wrappers "nav_tf_direct_relay.py"
kill_ros_wrappers "odom_tf_republisher.py"
kill_ros_wrappers "nav_static_tf_bootstrap.py"
kill_ros_wrappers "nav_identity_tf_publisher.py"
kill_ros_wrappers "relocalization_tf_republisher.py"
kill_ros_wrappers "safe_cmd_vel_bridge.py"
kill_ros_wrappers "ndt_goal_controller.py"
kill_ros_wrappers "nav_goal_marker_from_plan.py"
kill_ros_wrappers "nav_obstacle_block_alarm.py"
kill_ros_wrappers "aligned_nav_goal_adapter.py"
kill_ros_wrappers "nav_motion_safety_gate.py"
kill_ros_wrappers "nav_obstacle_cloud_republisher.py"
kill_ros_wrappers "cloud_callback_probe"
kill_ros_wrappers "cloud_geometry_inspector"
kill_ros_wrappers "static_transform_publisher.*nav_lidar"
kill_ros_wrappers "ros2 bag play"
kill_ros_wrappers "ros2 bag record"
kill_ros_wrappers "livox_ros_driver2_node"
kill_ros_wrappers "fastlio_mapping"
kill_ros_wrappers "pcd_ndt_localizer"
kill_ros_wrappers "nav2_map_server/map_server"
kill_ros_wrappers "planner_server"
kill_ros_wrappers "controller_server"
kill_ros_wrappers "smoother_server"
kill_ros_wrappers "behavior_server"
kill_ros_wrappers "bt_navigator"
kill_ros_wrappers "waypoint_follower"
kill_ros_wrappers "velocity_smoother"
kill_ros_wrappers "lifecycle_manager"
kill_ros_wrappers "ranger_base_node"
kill_ros_wrappers "ros2 topic hz"
kill_ros_wrappers "ros2 topic echo"

pkill -x fastlio_mapping 2>/dev/null || true
pkill -x livox_ros_driver2_node 2>/dev/null || true
pkill -x pcd_ndt_localizer 2>/dev/null || true
pkill -x map_server 2>/dev/null || true
pkill -x planner_server 2>/dev/null || true
pkill -x controller_server 2>/dev/null || true
pkill -x smoother_server 2>/dev/null || true
pkill -x behavior_server 2>/dev/null || true
pkill -x bt_navigator 2>/dev/null || true
pkill -x waypoint_follower 2>/dev/null || true
pkill -x velocity_smoother 2>/dev/null || true
pkill -x velocity_smooth 2>/dev/null || true
pkill -f nav2_collision_monitor/collision_monitor 2>/dev/null || true
pkill -x lifecycle_manager 2>/dev/null || true
pkill -x lifecycle_manag 2>/dev/null || true
pkill -x component_container 2>/dev/null || true
pkill -x component_container_isolated 2>/dev/null || true
pkill -x ranger_base_node 2>/dev/null || true
pkill -f nav_identity_tf_publisher.py 2>/dev/null || true
pkill -f relocalization_tf_republisher.py 2>/dev/null || true
pkill -f map_odom_tf_bridge.py 2>/dev/null || true
pkill -x map_odom_tf_bridge 2>/dev/null || true
pkill -f nav_tf_direct_relay.py 2>/dev/null || true
pkill -f odom_tf_republisher.py 2>/dev/null || true
pkill -f safe_cmd_vel_bridge.py 2>/dev/null || true
pkill -f ndt_goal_controller.py 2>/dev/null || true
pkill -f nav_goal_marker_from_plan.py 2>/dev/null || true
pkill -f nav_obstacle_block_alarm.py 2>/dev/null || true
pkill -f aligned_nav_goal_adapter.py 2>/dev/null || true
pkill -f nav_motion_safety_gate.py 2>/dev/null || true
pkill -f nav_obstacle_cloud_republisher.py 2>/dev/null || true
pkill -f cloud_callback_probe 2>/dev/null || true
pkill -f cloud_geometry_inspector 2>/dev/null || true
pkill -f seeed_activate_nav2_after_initialpose.sh 2>/dev/null || true
pkill -x rviz2 2>/dev/null || true
force_kill_matches

sleep 2
echo "Remaining matching processes:"
ps -eo pid,comm,args | egrep "livox|fastlio|pcd_ndt|map_server|planner_server|controller_server|smoother_server|behavior_server|bt_navigator|waypoint_follower|velocity_smooth|lifecycle_manager|ranger|map_camera_init_bridge|ros2 bag|rviz2" | grep -v egrep || true
