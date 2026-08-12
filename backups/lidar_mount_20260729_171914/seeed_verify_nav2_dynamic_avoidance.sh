#!/usr/bin/env bash
set -euo pipefail

ROS_ROOT="${ROS_ROOT:-/home/seeed/ros2}"
ROS_ENV="${ROS_ENV:-${ROS_ROOT}/use_ros_env.sh}"
NAV_FASTDDS_PROFILE="${NAV_FASTDDS_PROFILE:-${ROS_ROOT}/fastrtps_profile.xml}"
COSTMAP_INSPECTOR="${COSTMAP_INSPECTOR:-${ROS_ROOT}/scripts/inspect_nav_goal_costmap.py}"
OBSTACLE_TRANSFORM_VERIFIER="${OBSTACLE_TRANSFORM_VERIFIER:-${ROS_ROOT}/scripts/verify_nav_obstacle_transform.py}"
NAV2_LIFECYCLE="${NAV2_LIFECYCLE:-${ROS_ROOT}/scripts/nav2_manual_lifecycle.py}"
ACTIVATION_PID_FILE="${ROS_ROOT}/logs/nav2_activation.pid"
ACTIVATION_EXIT_FILE="${ROS_ROOT}/logs/nav2_activation.exit"
ACTIVATION_STATUS_FILE="${ROS_ROOT}/logs/nav2_activation.status"
EXPECTED_LIDAR_X_M="${EXPECTED_LIDAR_X_M:-0.890}"
EXPECTED_LIDAR_Y_M="${EXPECTED_LIDAR_Y_M:--0.050}"
EXPECTED_LIDAR_Z_M="${EXPECTED_LIDAR_Z_M:-0.700}"
EXPECTED_LIDAR_ROLL_DEG="${EXPECTED_LIDAR_ROLL_DEG:-15.732}"
EXPECTED_LIDAR_PITCH_DEG="${EXPECTED_LIDAR_PITCH_DEG:--0.611}"
EXPECTED_LIDAR_YAW_DEG="${EXPECTED_LIDAR_YAW_DEG:-90.0}"

run_ros() {
  "${ROS_ENV}" env FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" "$@"
}

failed=false
managed_nodes=(
  map_server planner_server controller_server smoother_server behavior_server
  bt_navigator waypoint_follower velocity_smoother collision_monitor
)

activation_pid=""
if [[ -r "${ACTIVATION_PID_FILE}" ]]; then
  activation_pid="$(tr -d '[:space:]' < "${ACTIVATION_PID_FILE}")"
fi
if [[ "${activation_pid}" =~ ^[0-9]+$ ]] \
  && kill -0 "${activation_pid}" 2>/dev/null; then
  cat "${ACTIVATION_STATUS_FILE}" 2>/dev/null || true
  echo "REFUSED: Nav2 activation is still running (pid=${activation_pid})." >&2
    echo "Run seeed_nav2_activation_job.sh wait 480 before verification." >&2
  exit 2
fi
if [[ -r "${ACTIVATION_EXIT_FILE}" ]] \
  && [[ "$(tr -d '[:space:]' < "${ACTIVATION_EXIT_FILE}")" != "0" ]]; then
  cat "${ACTIVATION_STATUS_FILE}" 2>/dev/null || true
  echo "REFUSED: the latest Nav2 activation did not exit successfully." >&2
  echo "Run seeed_nav2_activation_job.sh log 200 and keep the robot DISARMED." >&2
  exit 2
fi

echo "=== Nav2 lifecycle ==="
if ! timeout 45 "${ROS_ENV}" env \
  FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
  python3 "${NAV2_LIFECYCLE}" --check-only --timeout 8 \
  "${managed_nodes[@]}"; then
  failed=true
fi

echo "=== Dynamic obstacle cloud ==="
status="$(timeout 8 "${ROS_ENV}" env FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" ros2 topic echo --once \
  /nav_obstacle_cloud_status 2>/dev/null || true)"
printf '%s\n' "${status:-no status}"
grep -q 'data: ok received=' <<<"${status}" || failed=true
grep -q 'frame=base_link' <<<"${status}" || {
  echo "Obstacle cloud must be pre-transformed into base_link." >&2
  failed=true
}
obstacle_age="$(sed -n 's/.*age=\([0-9.]*\)s.*/\1/p' <<<"${status}" | head -n 1)"
if [[ -z "${obstacle_age}" ]] \
  || ! awk -v age="${obstacle_age}" 'BEGIN {exit !(age <= 1.00)}'; then
  echo "Obstacle cloud is stale: age=${obstacle_age:-missing}s (limit 1.00s)" >&2
  failed=true
fi
if [[ -r "${OBSTACLE_TRANSFORM_VERIFIER}" ]]; then
  timeout 15 "${ROS_ENV}" env \
    FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
    python3 "${OBSTACLE_TRANSFORM_VERIFIER}" || failed=true
else
  echo "Missing obstacle transform verifier: ${OBSTACLE_TRANSFORM_VERIFIER}" >&2
  failed=true
fi

check_topic() {
  local topic="$1"
  local minimum_subscribers="${2:-0}"
  local info publishers subscribers
  info="$(timeout 8 "${ROS_ENV}" env FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" ros2 topic info "${topic}" 2>/dev/null || true)"
  publishers="$(awk '/Publisher count:/{print $3}' <<<"${info}")"
  subscribers="$(awk '/Subscription count:/{print $3}' <<<"${info}")"
  printf '%-38s publishers=%s subscribers=%s\n' \
    "${topic}" "${publishers:-0}" "${subscribers:-0}"
  if (( ${publishers:-0} < 1 || ${subscribers:-0} < minimum_subscribers )); then
    failed=true
  fi
}

get_param_with_retry() {
  local node="$1"
  local parameter="$2"
  local value=""
  local attempt
  for attempt in $(seq 1 6); do
    value="$(timeout 8 "${ROS_ENV}" env \
      FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
      ros2 param get "${node}" "${parameter}" 2>/dev/null || true)"
    if [[ -n "${value}" && "${value}" != *"Node not found"* ]]; then
      printf '%s\n' "${value}"
      return 0
    fi
    sleep 2
  done
  printf '%s\n' "${value:-missing}"
  return 1
}

echo "=== Topic chain ==="
check_topic /nav_obstacle_cloud 3
check_topic /local_costmap/costmap 0
check_topic /global_costmap/costmap 0
check_topic /nav_goal_markers 0
check_topic /nav_obstacle_alarm 0
check_topic /nav_obstacle_alarm_status 0
check_topic /nav_obstacle_alarm_markers 0
check_topic /aligned_goal_status 0
check_topic /aligned_goal_approach_pose 0
check_topic /dynamic_obstacle_slow_zone 0
check_topic /dynamic_obstacle_stop_zone 0
check_topic /cmd_vel_nav 1
check_topic /cmd_vel_nav_smoothed 1
check_topic /cmd_vel_nav_collision_safe 1
check_topic /cmd_vel 1

echo "=== Direction-aligned goal adapter ==="
adapter_nodes="$(run_ros ros2 node list 2>/dev/null || true)"
actions="$(run_ros ros2 action list 2>/dev/null || true)"
printf '%s\n' "${actions}"
grep -qx '/aligned_nav_goal_adapter' <<<"${adapter_nodes}" || failed=true
grep -qx '/navigate_to_pose' <<<"${actions}" || failed=true
grep -qx '/navigate_to_pose_raw' <<<"${actions}" || failed=true
grep -qx '/navigate_through_poses' <<<"${actions}" || failed=true
if grep -qx '/ndt_goal_controller' <<<"${adapter_nodes}"; then
  echo "Conflicting legacy /ndt_goal_controller is still running." >&2
  failed=true
fi
smoothed_info="$(run_ros ros2 topic info /cmd_vel_nav_smoothed 2>/dev/null || true)"
smoothed_publishers="$(awk '/Publisher count:/{print $3}' <<<"${smoothed_info}")"
if [[ "${smoothed_publishers:-0}" != "1" ]]; then
  echo "/cmd_vel_nav_smoothed must have exactly one publisher; got ${smoothed_publishers:-0}." >&2
  failed=true
fi

echo "=== Safety state ==="
gate_status="$(timeout 8 "${ROS_ENV}" env FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" ros2 topic echo --once \
  /nav_motion_status 2>/dev/null || true)"
printf '%s\n' "${gate_status:-no gate status}"
if [[ -z "${gate_status}" ]]; then
  failed=true
fi
arm_service="$(timeout 8 "${ROS_ENV}" env FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
  ros2 service type /set_nav_motion_enabled 2>/dev/null || true)"
printf 'arm service: %s\n' "${arm_service:-missing}"
if [[ "${arm_service}" != "std_srvs/srv/SetBool" ]]; then
  failed=true
fi
legacy_arm_topic="$(timeout 8 "${ROS_ENV}" env FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
  ros2 topic info /nav_motion_enable 2>/dev/null || true)"
if grep -q 'Subscription count: [1-9]' <<<"${legacy_arm_topic}"; then
  echo "Legacy /nav_motion_enable subscriber is still active." >&2
  failed=true
fi

echo "=== Vehicle geometry parameters ==="
local_footprint="$(get_param_with_retry /local_costmap/local_costmap footprint || true)"
global_footprint="$(get_param_with_retry /global_costmap/global_costmap footprint || true)"
rotation_zone="$(get_param_with_retry /collision_monitor RotationStopZone.points || true)"
printf 'local footprint: %s\n' "${local_footprint:-missing}"
printf 'global footprint: %s\n' "${global_footprint:-missing}"
printf 'rotation stop zone: %s\n' "${rotation_zone:-missing}"
chassis_footprint='[[-0.38, 0.27], [0.38, 0.27], [0.38, -0.27], [-0.38, -0.27]]'
grep -Fq "${chassis_footprint}" <<<"${local_footprint}" || failed=true
grep -Fq "${chassis_footprint}" <<<"${global_footprint}" || failed=true
grep -q '0.6' <<<"${rotation_zone}" || failed=true

echo "=== No-spin arrival policy ==="
yaw_tolerance="$(get_param_with_retry /controller_server general_goal_checker.yaw_goal_tolerance || true)"
rotate_to_heading="$(get_param_with_retry /controller_server FollowPath.use_rotate_to_heading || true)"
allow_reversing="$(get_param_with_retry /controller_server FollowPath.allow_reversing || true)"
movement_radius="$(get_param_with_retry /controller_server progress_checker.required_movement_radius || true)"
bt_xml="$(get_param_with_retry /bt_navigator default_nav_to_pose_bt_xml || true)"
behavior_plugins="$(get_param_with_retry /behavior_server behavior_plugins || true)"
planner_plugin="$(get_param_with_retry /planner_server GridBased.plugin || true)"
motion_model="$(get_param_with_retry /planner_server GridBased.motion_model_for_search || true)"
turning_radius="$(get_param_with_retry /planner_server GridBased.minimum_turning_radius || true)"
collision_source_timeout="$(get_param_with_retry /collision_monitor source_timeout || true)"
printf 'yaw tolerance: %s\n' "${yaw_tolerance:-missing}"
printf 'rotate to heading: %s\n' "${rotate_to_heading:-missing}"
printf 'allow reversing: %s\n' "${allow_reversing:-missing}"
printf 'progress radius: %s\n' "${movement_radius:-missing}"
printf 'BT XML: %s\n' "${bt_xml:-missing}"
printf 'behavior plugins: %s\n' "${behavior_plugins:-missing}"
printf 'planner plugin: %s\n' "${planner_plugin:-missing}"
printf 'motion model: %s\n' "${motion_model:-missing}"
printf 'minimum turning radius: %s\n' "${turning_radius:-missing}"
printf 'collision source timeout: %s\n' "${collision_source_timeout:-missing}"
grep -q '0.17' <<<"${yaw_tolerance}" || failed=true
grep -q 'False' <<<"${rotate_to_heading}" || failed=true
grep -q 'True' <<<"${allow_reversing}" || failed=true
grep -q '0.1' <<<"${movement_radius}" || failed=true
grep -q 'navigate_to_pose_no_spin.xml' <<<"${bt_xml}" || failed=true
grep -q 'SmacPlannerHybrid' <<<"${planner_plugin}" || failed=true
grep -q 'REEDS_SHEPP' <<<"${motion_model}" || failed=true
grep -q '0.45' <<<"${turning_radius}" || failed=true
grep -q '1.5' <<<"${collision_source_timeout}" || failed=true
if grep -Eq 'spin|backup|drive_on_heading' <<<"${behavior_plugins}"; then
  echo "Unsafe autonomous rotation/reverse behavior is still configured." >&2
  failed=true
fi

echo "=== LiDAR mounting transform ==="
timeout 6 "${ROS_ENV}" env FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
  ros2 run tf2_ros tf2_echo base_link nav_lidar \
  >/tmp/nav_lidar_geometry.txt 2>&1 || true
cat /tmp/nav_lidar_geometry.txt
tf_translation="$(awk '
  /Translation: \[/{gsub(/[][]/, ""); gsub(/,/, ""); print $3, $4, $5; exit}
' /tmp/nav_lidar_geometry.txt)"
tf_rpy_deg="$(awk '
  /RPY \(degree\) \[/{gsub(/[][]/, ""); gsub(/,/, ""); print $6, $7, $8; exit}
' /tmp/nav_lidar_geometry.txt)"
if ! awk -v xyz="${tf_translation}" -v rpy="${tf_rpy_deg}" \
  -v expected_x="${EXPECTED_LIDAR_X_M}" \
  -v expected_y="${EXPECTED_LIDAR_Y_M}" \
  -v expected_z="${EXPECTED_LIDAR_Z_M}" \
  -v expected_roll="${EXPECTED_LIDAR_ROLL_DEG}" \
  -v expected_pitch="${EXPECTED_LIDAR_PITCH_DEG}" \
  -v expected_yaw="${EXPECTED_LIDAR_YAW_DEG}" '
  BEGIN {
    split(xyz, p, " "); split(rpy, a, " ");
    dx = p[1] - expected_x; if (dx < 0) dx = -dx;
    dy = p[2] - expected_y; if (dy < 0) dy = -dy;
    dz = p[3] - expected_z; if (dz < 0) dz = -dz;
    droll = a[1] - expected_roll; if (droll < 0) droll = -droll;
    dpitch = a[2] - expected_pitch; if (dpitch < 0) dpitch = -dpitch;
    yaw = a[3]; if (yaw < 0) yaw = -yaw;
    dyaw = yaw - expected_yaw; if (dyaw < 0) dyaw = -dyaw;
    exit !(dx <= 0.002 && dy <= 0.002 && dz <= 0.002 &&
      droll <= 0.10 && dpitch <= 0.10 && dyaw <= 1.0);
  }'; then
  echo "LiDAR TF mismatch: expected xyz=(${EXPECTED_LIDAR_X_M},${EXPECTED_LIDAR_Y_M},${EXPECTED_LIDAR_Z_M}), rpy=(${EXPECTED_LIDAR_ROLL_DEG},${EXPECTED_LIDAR_PITCH_DEG},${EXPECTED_LIDAR_YAW_DEG}) deg" >&2
  failed=true
fi

echo "=== Navigation start cell ==="
if [[ -r "${COSTMAP_INSPECTOR}" ]]; then
  timeout 25 "${ROS_ENV}" env FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
    python3 "${COSTMAP_INSPECTOR}" --require-start-passable --timeout 18 \
    || failed=true
else
  echo "Missing costmap inspector: ${COSTMAP_INSPECTOR}" >&2
  failed=true
fi

echo "=== Costmap samples ==="
timeout 10 "${ROS_ENV}" env FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" ros2 topic echo --once \
  /local_costmap/costmap --field info >/tmp/local_costmap_info.txt 2>&1 || failed=true
timeout 10 "${ROS_ENV}" env FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" ros2 topic echo --once \
  /global_costmap/costmap --field info >/tmp/global_costmap_info.txt 2>&1 || failed=true
cat /tmp/local_costmap_info.txt 2>/dev/null || true
cat /tmp/global_costmap_info.txt 2>/dev/null || true

if [[ "${failed}" == true ]]; then
  echo "RESULT: FAILED - keep the robot DISARMED." >&2
  exit 1
fi

echo "RESULT: PASS - Nav2 dynamic avoidance data path is complete."
echo "This check does not arm or move the robot."
