#!/usr/bin/env bash
set -euo pipefail

ROS_ROOT="${ROS_ROOT:-/home/seeed/ros2}"
ROS_ENV="${ROS_ENV:-${ROS_ROOT}/use_ros_env.sh}"
NAV2_PARAMS="${NAV2_PARAMS:-${ROS_ROOT}/nav2/nav2_pcd_ndt_manual_002.yaml}"
NAV_YAML="${1:-${ROS_ROOT}/maps/replay/fastlio_map_manual_001_level_groundsafe_20260729_nav.yaml}"

if [[ ! -x "${ROS_ENV}" ]]; then
  echo "ROS env script not executable: ${ROS_ENV}" >&2
  exit 1
fi
if [[ ! -f "${NAV2_PARAMS}" ]]; then
  echo "Nav2 params not found: ${NAV2_PARAMS}" >&2
  exit 1
fi
if [[ ! -f "${NAV_YAML}" ]]; then
  echo "Nav map YAML not found: ${NAV_YAML}" >&2
  exit 1
fi

mkdir -p "${ROS_ROOT}/logs"

echo "Stopping old Nav2/map verification processes only..."
ps -eo pid=,comm=,args= | awk '
  /nav2_bringup|map_server|planner_server|controller_server|smoother_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|velocity_smooth|lifecycle_manag|static_transform_publisher|nav_identity_tf_publisher.py/ &&
  $0 !~ /awk/ &&
  $0 !~ /seeed_verify_nav2_chain_safe/ {print $1}
' | xargs -r kill -9 2>/dev/null || true
sleep 2

echo "Starting dynamic TF for dry verification: map -> relocalization_body"
nohup "${ROS_ENV}" python3 "${ROS_ROOT}/scripts/nav_identity_tf_publisher.py" \
  > "${ROS_ROOT}/logs/nav2_verify_identity_tf.log" 2>&1 &
sleep 2

echo "Starting map_server with: ${NAV_YAML}"
nohup "${ROS_ENV}" ros2 run nav2_map_server map_server --ros-args \
  -p use_sim_time:=false \
  -p yaml_filename:="${NAV_YAML}" \
  -p topic_name:=/map \
  -p frame_id:=map \
  > "${ROS_ROOT}/logs/nav2_verify_map_server.log" 2>&1 &
sleep 4
"${ROS_ENV}" ros2 lifecycle set /map_server configure
sleep 1
"${ROS_ENV}" ros2 lifecycle set /map_server activate

echo "Starting Nav2 nodes in safe mode. Output is /cmd_vel_nav, not /cmd_vel."
nohup "${ROS_ENV}" ros2 run nav2_controller controller_server --ros-args \
  --params-file "${NAV2_PARAMS}" \
  -r cmd_vel:=/cmd_vel_nav \
  -r /cmd_vel:=/cmd_vel_nav \
  > "${ROS_ROOT}/logs/nav2_verify_controller_server.log" 2>&1 &

nohup "${ROS_ENV}" ros2 run nav2_smoother smoother_server --ros-args \
  --params-file "${NAV2_PARAMS}" \
  > "${ROS_ROOT}/logs/nav2_verify_smoother_server.log" 2>&1 &

nohup "${ROS_ENV}" ros2 run nav2_planner planner_server --ros-args \
  --params-file "${NAV2_PARAMS}" \
  > "${ROS_ROOT}/logs/nav2_verify_planner_server.log" 2>&1 &

nohup "${ROS_ENV}" ros2 run nav2_behaviors behavior_server --ros-args \
  --params-file "${NAV2_PARAMS}" \
  -r cmd_vel:=/cmd_vel_nav \
  -r /cmd_vel:=/cmd_vel_nav \
  > "${ROS_ROOT}/logs/nav2_verify_behavior_server.log" 2>&1 &

nohup "${ROS_ENV}" ros2 run nav2_bt_navigator bt_navigator --ros-args \
  --params-file "${NAV2_PARAMS}" \
  > "${ROS_ROOT}/logs/nav2_verify_bt_navigator.log" 2>&1 &

nohup "${ROS_ENV}" ros2 run nav2_waypoint_follower waypoint_follower --ros-args \
  --params-file "${NAV2_PARAMS}" \
  > "${ROS_ROOT}/logs/nav2_verify_waypoint_follower.log" 2>&1 &

nohup "${ROS_ENV}" ros2 run nav2_velocity_smoother velocity_smoother --ros-args \
  --params-file "${NAV2_PARAMS}" \
  -r cmd_vel:=/cmd_vel_nav \
  -r cmd_vel_smoothed:=/cmd_vel_nav_smoothed \
  > "${ROS_ROOT}/logs/nav2_verify_velocity_smoother.log" 2>&1 &

sleep 8
managed_nodes=(
  planner_server
  controller_server
  smoother_server
  behavior_server
  bt_navigator
  waypoint_follower
  velocity_smoother
)

echo "Configuring lifecycle nodes..."
for node in "${managed_nodes[@]}"; do
  timeout 45 "${ROS_ENV}" ros2 service call "/${node}/change_state" lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}" >/tmp/nav2_verify_"${node}"_configure.txt
done

echo "Waiting for Nav2 TF buffers to receive map -> relocalization_body..."
sleep 10

echo "Activating lifecycle nodes..."
for node in "${managed_nodes[@]}"; do
  timeout 45 "${ROS_ENV}" ros2 service call "/${node}/change_state" lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}" >/tmp/nav2_verify_"${node}"_activate.txt
done

echo "Verification results:"
echo "--- lifecycle"
for node in map_server "${managed_nodes[@]}"; do
  timeout 8 "${ROS_ENV}" ros2 lifecycle get "/${node}" || true
done

echo "--- topics"
"${ROS_ENV}" ros2 topic list -t | egrep "map|costmap|plan|cmd_vel|goal|tf" || true

echo "--- map sample"
timeout 8 "${ROS_ENV}" ros2 topic echo --once /map >/tmp/nav2_verify_map.txt
head -n 20 /tmp/nav2_verify_map.txt

echo "--- action servers"
"${ROS_ENV}" ros2 action list | egrep "navigate|compute_path|follow_path|smooth_path" || true

echo "--- safety check: /cmd_vel must have no publishers from Nav2"
"${ROS_ENV}" ros2 topic info -v /cmd_vel || true
echo "--- Nav2 safe command topic"
"${ROS_ENV}" ros2 topic info -v /cmd_vel_nav || true
echo "--- Nav2 smoothed safe command topic"
"${ROS_ENV}" ros2 topic info -v /cmd_vel_nav_smoothed || true

echo "Safe Nav2 chain verification finished. No navigation goal was sent."
