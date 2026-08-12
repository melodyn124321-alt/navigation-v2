#!/usr/bin/env bash
set -euo pipefail

ROS_ROOT="${ROS_ROOT:-/home/seeed/ros2}"
ROS_ENV="${ROS_ENV:-${ROS_ROOT}/use_ros_env.sh}"
CONFIRM_NAV_MOTION="${CONFIRM_NAV_MOTION:-}"
NAV_FASTDDS_PROFILE="${NAV_FASTDDS_PROFILE:-${ROS_ROOT}/fastrtps_profile.xml}"

mkdir -p "${ROS_ROOT}/logs"

if [[ ! -r "${NAV_FASTDDS_PROFILE}" ]]; then
  echo "Missing navigation FastDDS profile: ${NAV_FASTDDS_PROFILE}" >&2
  exit 1
fi

if ! timeout 8 "${ROS_ENV}" ros2 topic echo --once /relocalization_pose >/tmp/ndt_goal_pose_check.txt 2>&1; then
  cat /tmp/ndt_goal_pose_check.txt >&2 || true
  echo "No /relocalization_pose. Start LiDAR/NDT and set RViz 2D Pose Estimate first." >&2
  exit 1
fi

ps -eo pid=,args= | awk '
  /ndt_goal_controller.py|nav_motion_safety_gate.py/ &&
  $0 !~ /awk/ &&
  $0 !~ /seeed_start_ndt_goal_nav_safe/ {print $1}
' | xargs -r kill 2>/dev/null || true
sleep 1

start_navigation_nodes() {
  echo "Starting NDT goal controller: /goal_pose -> /cmd_vel_nav_smoothed"
  setsid -f "${ROS_ENV}" env FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
    python3 "${ROS_ROOT}/scripts/ndt_goal_controller.py" --ros-args \
    -p localization_to_base_yaw_rad:=-1.570796327 \
    -p localization_to_base_x_m:=0.042528450 \
    -p localization_to_base_y_m:=0.666725193 \
    -p localization_timeout:=1.0 \
    -p goal_tolerance:=0.08 \
    -p goal_label_offset_x:=0.30 \
    -p goal_label_offset_y:=0.30 \
    -p align_final_heading:=false \
    -p yaw_tolerance:=0.17 \
    -p final_yaw_stable_time:=0.60 \
    -p final_yaw_timeout:=45.0 \
    -p max_final_angular:=0.20 \
    -p k_final_angular:=0.60 \
    -p final_yaw_filter_alpha:=0.25 \
    -p max_linear:=0.08 \
    -p max_angular:=0.18 \
    > "${ROS_ROOT}/logs/ndt_goal_controller.log" 2>&1 < /dev/null

  # FastDDS on this host can allocate the same UDP participant slot when two
  # Python nodes start in the same instant.  Let the controller register first.
  sleep 3

  echo "Starting motion safety gate DISARMED: /cmd_vel_nav_smoothed -> /cmd_vel"
  setsid -f "${ROS_ENV}" env FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
    python3 "${ROS_ROOT}/scripts/nav_motion_safety_gate.py" --ros-args \
    -p localization_timeout:=10.0 \
    -p max_angular:=0.18 \
    -p minimum_linear_for_turn:=0.012 \
    -p max_motion_curvature:=2.25 \
    -p curvature_slack:=0.015 \
    > "${ROS_ROOT}/logs/nav_motion_safety_gate.log" 2>&1 < /dev/null
}

navigation_graph_ready() {
  local nodes cmd_info goal_info
  nodes="$("${ROS_ENV}" ros2 node list 2>/dev/null || true)"
  [[ "${nodes}" == *"/ndt_goal_controller"* ]] || return 1
  [[ "${nodes}" == *"/nav_motion_safety_gate"* ]] || return 1
  cmd_info="$("${ROS_ENV}" ros2 topic info /cmd_vel_nav_smoothed 2>/dev/null || true)"
  goal_info="$("${ROS_ENV}" ros2 topic info /goal_pose 2>/dev/null || true)"
  grep -q 'Publisher count: [1-9]' <<<"${cmd_info}" || return 1
  grep -q 'Subscription count: [1-9]' <<<"${goal_info}" || return 1
}

ready=false
for attempt in 1 2; do
  start_navigation_nodes
  for check in {1..10}; do
    if navigation_graph_ready; then
      echo "Navigation ROS graph ready (attempt ${attempt}, check ${check})."
      ready=true
      break 2
    fi
    sleep 1
  done
  echo "Navigation ROS graph did not become ready; restarting once." >&2
  ps -eo pid=,args= | awk '
    /ndt_goal_controller.py|nav_motion_safety_gate.py/ && $0 !~ /awk/ {print $1}
  ' | xargs -r kill 2>/dev/null || true
  sleep 2
done

if [[ "${ready}" != true ]]; then
  echo "Navigation nodes are alive but missing from the ROS graph." >&2
  tail -n 30 "${ROS_ROOT}/logs/ndt_goal_controller.log" >&2 || true
  tail -n 30 "${ROS_ROOT}/logs/nav_motion_safety_gate.log" >&2 || true
  exit 1
fi

if [[ "${CONFIRM_NAV_MOTION}" == "I_UNDERSTAND" ]]; then
  echo "Operator confirmation present. You may arm with seeed_arm_nav_motion.sh after checking RViz."
else
  echo "Safe mode: gate is DISARMED. The robot will not move until you arm it."
fi

echo "Expected topics:"
"${ROS_ENV}" ros2 topic list -t | egrep "goal_pose|ndt_goal|cmd_vel|nav_motion|relocalization" || true

echo "Controller status:"
timeout 5 "${ROS_ENV}" ros2 topic echo --once /ndt_goal_status || true
echo "Motion gate status:"
timeout 5 "${ROS_ENV}" ros2 topic echo --once /nav_motion_status || true
