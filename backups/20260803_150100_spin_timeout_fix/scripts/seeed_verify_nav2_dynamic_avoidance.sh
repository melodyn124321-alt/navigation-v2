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
EXPECTED_LIDAR_X_M="${EXPECTED_LIDAR_X_M:-0.100}"
EXPECTED_LIDAR_Y_M="${EXPECTED_LIDAR_Y_M:--0.110}"
EXPECTED_LIDAR_Z_M="${EXPECTED_LIDAR_Z_M:-0.749}"
EXPECTED_LIDAR_ROLL_DEG="${EXPECTED_LIDAR_ROLL_DEG:--45.937}"
EXPECTED_LIDAR_PITCH_DEG="${EXPECTED_LIDAR_PITCH_DEG:--0.4065}"
EXPECTED_LIDAR_YAW_DEG="${EXPECTED_LIDAR_YAW_DEG:--90.0}"
EXPECTED_SONAR_X_M="${EXPECTED_SONAR_X_M:--0.160}"
EXPECTED_SONAR_LEFT_Y_M="${EXPECTED_SONAR_LEFT_Y_M:-0.180}"
EXPECTED_SONAR_RIGHT_Y_M="${EXPECTED_SONAR_RIGHT_Y_M:--0.180}"
EXPECTED_SONAR_Z_M="${EXPECTED_SONAR_Z_M:-0.545}"
EXPECTED_SONAR_YAW_DEG="${EXPECTED_SONAR_YAW_DEG:-180.0}"

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
  local require_publisher="${3:-true}"
  local info publishers subscribers
  info="$(timeout 8 "${ROS_ENV}" env FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" ros2 topic info "${topic}" 2>/dev/null || true)"
  publishers="$(awk '/Publisher count:/{print $3}' <<<"${info}")"
  subscribers="$(awk '/Subscription count:/{print $3}' <<<"${info}")"
  printf '%-38s publishers=%s subscribers=%s\n' \
    "${topic}" "${publishers:-0}" "${subscribers:-0}"
  if (( ${subscribers:-0} < minimum_subscribers )); then
    failed=true
  fi
  if [[ "${require_publisher}" == "true" ]] \
    && (( ${publishers:-0} < 1 )); then
    failed=true
  fi
  if [[ "${require_publisher}" != "true" ]] \
    && (( ${publishers:-0} < 1 )); then
    echo "  ${topic}: no live publisher; acceptable while motion remains DISARMED."
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
check_topic /dynamic_obstacle_front_stop_zone 0
check_topic /cmd_vel_nav 1
check_topic /cmd_vel_nav_smoothed 1
check_topic /cmd_vel_nav_collision_safe 1
check_topic /cmd_vel 1
check_topic /nav_reverse_path_policy 0
check_topic /rear_ultrasonic_safety_status 0
check_topic /rear_ultrasonic_reverse_allowed 0
check_topic /rear_ultrasonic_left_turn_allowed 0
check_topic /rear_ultrasonic_right_turn_allowed 0
check_topic /rviz_goal_pose_bridge_status 0
check_topic /goal_pose 1
# The HN console owns this heartbeat. Static obstacle-pipeline verification is
# allowed before that console starts; the arm checker still requires a fresh
# heartbeat and refuses physical motion without it.
check_topic /hn_nav_operator_heartbeat 1 false
check_topic /hn_nav_operator_link_status 0
check_topic /ultrasonic/sensor_1/range 1
check_topic /ultrasonic/sensor_2/range 1
check_topic /ultrasonic/healthy 1

echo "=== Direction-aligned goal adapter ==="
adapter_nodes="$(run_ros ros2 node list 2>/dev/null || true)"
actions="$(run_ros ros2 action list 2>/dev/null || true)"
printf '%s\n' "${actions}"
grep -qx '/aligned_nav_goal_adapter' <<<"${adapter_nodes}" || failed=true
grep -qx '/rviz_goal_pose_bridge' <<<"${adapter_nodes}" || failed=true
grep -qx '/aligned_navigate_to_pose' <<<"${actions}" || failed=true
grep -qx '/navigate_to_pose' <<<"${actions}" || failed=true
grep -qx '/navigate_to_pose_raw' <<<"${actions}" || failed=true
grep -qx '/navigate_through_poses' <<<"${actions}" || failed=true
grep -qx '/spin' <<<"${actions}" || failed=true
if grep -qx '/ndt_goal_controller' <<<"${adapter_nodes}"; then
  echo "Conflicting legacy /ndt_goal_controller is still running." >&2
  failed=true
fi
goal_bridge_status="$(timeout 8 "${ROS_ENV}" env \
  FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
  ros2 topic echo --once /rviz_goal_pose_bridge_status \
  std_msgs/msg/String 2>/dev/null || true)"
printf '%s\n' "${goal_bridge_status:-no RViz goal bridge status}"
grep -q 'data: READY action_server=true' <<<"${goal_bridge_status}" \
  || failed=true
goal_pose_endpoints="$(run_ros ros2 topic info /goal_pose -v 2>/dev/null || true)"
printf '%s\n' "${goal_pose_endpoints}"
if ! grep -q '^Subscription count: 1$' <<<"${goal_pose_endpoints}" \
  || ! grep -q '^Node name: rviz_goal_pose_bridge$' \
    <<<"${goal_pose_endpoints}" \
  || grep -q '^Node name: bt_navigator$' <<<"${goal_pose_endpoints}"; then
  echo "/goal_pose must be consumed only by rviz_goal_pose_bridge." >&2
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
sonar_status="$(timeout 8 "${ROS_ENV}" env FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" ros2 topic echo --once \
  --full-length /rear_ultrasonic_safety_status 2>/dev/null || true)"
printf '%s\n' "${sonar_status:-no rear ultrasonic safety status}"
grep -q 'raw_m=' <<<"${sonar_status}" || failed=true
grep -q 'sensor_to_tail_m=0.200' <<<"${sonar_status}" || failed=true
grep -q 'reverse_allowed=' <<<"${sonar_status}" || failed=true
grep -q 'left_turn_allowed=' <<<"${sonar_status}" || failed=true
grep -q 'right_turn_allowed=' <<<"${sonar_status}" || failed=true
grep -q 'reverse_speed_scale=' <<<"${sonar_status}" || failed=true
grep -q 'blocking_sides=' <<<"${sonar_status}" || failed=true
grep -q 'reverse_blocking_sides=' <<<"${sonar_status}" || failed=true
grep -q 'turn_blocking_sides=' <<<"${sonar_status}" || failed=true
grep -q 'turn_allow_m=0.250' <<<"${sonar_status}" || failed=true
grep -q 'turn_guard=\[left:both,right:both,reverse:both\]' \
  <<<"${sonar_status}" || failed=true
grep -q 'classification=' <<<"${sonar_status}" || failed=true
grep -q 'detect_m=\[0.080,2.000\]' <<<"${sonar_status}" || failed=true
grep -q 'self_echo_m=\[0.160,0.220\]' <<<"${sonar_status}" || failed=true
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
front_stop_zone="$(get_param_with_retry /collision_monitor FrontStopZone.points || true)"
collision_polygons="$(get_param_with_retry /collision_monitor polygons || true)"
printf 'local footprint: %s\n' "${local_footprint:-missing}"
printf 'global footprint: %s\n' "${global_footprint:-missing}"
printf 'rotation stop zone: %s\n' "${rotation_zone:-missing}"
printf 'front stop zone: %s\n' "${front_stop_zone:-missing}"
printf 'collision polygons: %s\n' "${collision_polygons:-missing}"
chassis_footprint='[[-0.38, 0.27], [0.38, 0.27], [0.38, -0.27], [-0.38, -0.27]]'
grep -Fq "${chassis_footprint}" <<<"${local_footprint}" || failed=true
grep -Fq "${chassis_footprint}" <<<"${global_footprint}" || failed=true
grep -q '0.6' <<<"${rotation_zone}" || failed=true
grep -q '0.9' <<<"${front_stop_zone}" || failed=true
grep -q 'FrontStopZone' <<<"${collision_polygons}" || failed=true

echo "=== No-spin arrival policy ==="
yaw_tolerance="$(get_param_with_retry /controller_server general_goal_checker.yaw_goal_tolerance || true)"
rotate_to_heading="$(get_param_with_retry /controller_server FollowPath.use_rotate_to_heading || true)"
allow_reversing="$(get_param_with_retry /controller_server FollowPath.allow_reversing || true)"
movement_radius="$(get_param_with_retry /controller_server progress_checker.required_movement_radius || true)"
movement_allowance="$(get_param_with_retry /controller_server progress_checker.movement_time_allowance || true)"
controller_failure_tolerance="$(get_param_with_retry /controller_server failure_tolerance || true)"
local_costmap_frame="$(get_param_with_retry /local_costmap/local_costmap global_frame || true)"
adapter_progress_timeout="$(get_param_with_retry /aligned_nav_goal_adapter progress_timeout || true)"
adapter_progress_displacement="$(get_param_with_retry /aligned_nav_goal_adapter progress_min_displacement || true)"
adapter_small_spin_max="$(get_param_with_retry /aligned_nav_goal_adapter small_spin_max_angle || true)"
adapter_alignment_max="$(get_param_with_retry /aligned_nav_goal_adapter direct_alignment_max_angle || true)"
adapter_automatic_reverse="$(get_param_with_retry /aligned_nav_goal_adapter automatic_reverse_enabled || true)"
adapter_final_alignment_max="$(get_param_with_retry /aligned_nav_goal_adapter final_alignment_max_angle || true)"
adapter_reverse_allowance="$(get_param_with_retry /aligned_nav_goal_adapter reverse_time_allowance_factor || true)"
adapter_terminal_handoff="$(get_param_with_retry /aligned_nav_goal_adapter terminal_handoff_distance || true)"
adapter_terminal_tolerance="$(get_param_with_retry /aligned_nav_goal_adapter terminal_position_tolerance || true)"
adapter_terminal_allowance="$(get_param_with_retry /aligned_nav_goal_adapter terminal_time_allowance_factor || true)"
bt_xml="$(get_param_with_retry /bt_navigator default_nav_to_pose_bt_xml || true)"
behavior_plugins="$(get_param_with_retry /behavior_server behavior_plugins || true)"
behavior_frame="$(get_param_with_retry /behavior_server global_frame || true)"
planner_plugin="$(get_param_with_retry /planner_server GridBased.plugin || true)"
collision_source_timeout="$(get_param_with_retry /collision_monitor source_timeout || true)"
slowdown_ratio="$(get_param_with_retry /collision_monitor SlowZone.slowdown_ratio || true)"
slowdown_max_points="$(get_param_with_retry /collision_monitor SlowZone.max_points || true)"
reverse_straight_only="$(get_param_with_retry /nav_motion_safety_gate reverse_straight_only || true)"
reverse_plan_topic="$(get_param_with_retry /nav_motion_safety_gate plan_topic || true)"
reverse_plan_timeout="$(get_param_with_retry /nav_motion_safety_gate plan_timeout || true)"
straight_lateral_limit="$(get_param_with_retry /nav_motion_safety_gate straight_path_max_lateral_error || true)"
straight_heading_limit="$(get_param_with_retry /nav_motion_safety_gate straight_path_max_heading_span || true)"
straight_confirmations="$(get_param_with_retry /nav_motion_safety_gate straight_path_confirmations_required || true)"
allow_small_spin="$(get_param_with_retry /nav_motion_safety_gate allow_small_in_place_rotation || true)"
gate_small_spin_max="$(get_param_with_retry /nav_motion_safety_gate max_in_place_rotation || true)"
gate_small_spin_rate="$(get_param_with_retry /nav_motion_safety_gate max_in_place_angular || true)"
gate_small_spin_reset="$(get_param_with_retry /nav_motion_safety_gate small_spin_reset_distance || true)"
sonar_stop_distance="$(get_param_with_retry /nav_motion_safety_gate ultrasonic_stop_distance || true)"
sonar_clear_distance="$(get_param_with_retry /nav_motion_safety_gate ultrasonic_clear_distance || true)"
sonar_turn_allow_distance="$(get_param_with_retry /nav_motion_safety_gate ultrasonic_turn_allow_distance || true)"
sonar_self_echo_enabled="$(get_param_with_retry /nav_motion_safety_gate ultrasonic_self_echo_enabled || true)"
sonar_self_echo_min="$(get_param_with_retry /nav_motion_safety_gate ultrasonic_self_echo_min_distance || true)"
sonar_self_echo_max="$(get_param_with_retry /nav_motion_safety_gate ultrasonic_self_echo_max_distance || true)"
sonar_setback="$(get_param_with_retry /nav_motion_safety_gate ultrasonic_sensor_to_rear_edge || true)"
sonar_tail_clearance="$(get_param_with_retry /nav_motion_safety_gate ultrasonic_required_tail_clearance || true)"
sonar_hold="$(get_param_with_retry /nav_motion_safety_gate ultrasonic_block_hold_sec || true)"
sonar_clear_samples="$(get_param_with_retry /nav_motion_safety_gate ultrasonic_clear_samples_required || true)"
sonar_no_echo_samples="$(get_param_with_retry /nav_motion_safety_gate ultrasonic_no_echo_clear_samples_required || true)"
sonar_turn_guard="$(get_param_with_retry /nav_motion_safety_gate ultrasonic_turn_guard_enabled || true)"
sonar_reverse_taper="$(get_param_with_retry /nav_motion_safety_gate ultrasonic_reverse_speed_taper_enabled || true)"
operator_heartbeat_required="$(get_param_with_retry /nav_motion_safety_gate operator_heartbeat_required || true)"
operator_heartbeat_timeout="$(get_param_with_retry /nav_motion_safety_gate operator_heartbeat_timeout || true)"
printf 'yaw tolerance: %s\n' "${yaw_tolerance:-missing}"
printf 'rotate to heading: %s\n' "${rotate_to_heading:-missing}"
printf 'allow reversing: %s\n' "${allow_reversing:-missing}"
printf 'progress radius: %s\n' "${movement_radius:-missing}"
printf 'progress allowance: %s\n' "${movement_allowance:-missing}"
printf 'controller failure tolerance: %s\n' "${controller_failure_tolerance:-missing}"
printf 'local costmap frame: %s\n' "${local_costmap_frame:-missing}"
printf 'adapter progress timeout: %s\n' "${adapter_progress_timeout:-missing}"
printf 'adapter progress displacement: %s\n' "${adapter_progress_displacement:-missing}"
printf 'adapter small-spin max: %s\n' "${adapter_small_spin_max:-missing}"
printf 'adapter direct-alignment max: %s\n' "${adapter_alignment_max:-missing}"
printf 'adapter automatic reverse: %s\n' "${adapter_automatic_reverse:-missing}"
printf 'adapter final-alignment max: %s\n' "${adapter_final_alignment_max:-missing}"
printf 'adapter reverse allowance factor: %s\n' "${adapter_reverse_allowance:-missing}"
printf 'adapter terminal handoff: %s\n' "${adapter_terminal_handoff:-missing}"
printf 'adapter terminal tolerance: %s\n' "${adapter_terminal_tolerance:-missing}"
printf 'adapter terminal allowance factor: %s\n' "${adapter_terminal_allowance:-missing}"
printf 'BT XML: %s\n' "${bt_xml:-missing}"
printf 'behavior plugins: %s\n' "${behavior_plugins:-missing}"
printf 'behavior frame: %s\n' "${behavior_frame:-missing}"
printf 'planner plugin: %s\n' "${planner_plugin:-missing}"
printf 'collision source timeout: %s\n' "${collision_source_timeout:-missing}"
printf 'slow-zone ratio: %s\n' "${slowdown_ratio:-missing}"
printf 'slow-zone max points: %s\n' "${slowdown_max_points:-missing}"
printf 'reverse straight only: %s\n' "${reverse_straight_only:-missing}"
printf 'reverse plan topic: %s\n' "${reverse_plan_topic:-missing}"
printf 'reverse plan timeout: %s\n' "${reverse_plan_timeout:-missing}"
printf 'straight lateral limit: %s\n' "${straight_lateral_limit:-missing}"
printf 'straight heading limit: %s\n' "${straight_heading_limit:-missing}"
printf 'straight confirmations: %s\n' "${straight_confirmations:-missing}"
printf 'allow bounded small spin: %s\n' "${allow_small_spin:-missing}"
printf 'gate small-spin max: %s\n' "${gate_small_spin_max:-missing}"
printf 'gate small-spin rate: %s\n' "${gate_small_spin_rate:-missing}"
printf 'gate small-spin reset distance: %s\n' "${gate_small_spin_reset:-missing}"
printf 'rear sonar stop distance: %s\n' "${sonar_stop_distance:-missing}"
printf 'rear sonar clear distance: %s\n' "${sonar_clear_distance:-missing}"
printf 'rear sonar turn allow distance: %s\n' "${sonar_turn_allow_distance:-missing}"
printf 'rear sonar self-echo enabled: %s\n' "${sonar_self_echo_enabled:-missing}"
printf 'rear sonar self-echo minimum: %s\n' "${sonar_self_echo_min:-missing}"
printf 'rear sonar self-echo maximum: %s\n' "${sonar_self_echo_max:-missing}"
printf 'rear sonar setback: %s\n' "${sonar_setback:-missing}"
printf 'rear sonar tail clearance: %s\n' "${sonar_tail_clearance:-missing}"
printf 'rear sonar latch hold: %s\n' "${sonar_hold:-missing}"
printf 'rear sonar clear samples: %s\n' "${sonar_clear_samples:-missing}"
printf 'rear sonar no-echo clear samples: %s\n' "${sonar_no_echo_samples:-missing}"
printf 'rear sonar turn guard: %s\n' "${sonar_turn_guard:-missing}"
printf 'rear sonar reverse speed taper: %s\n' "${sonar_reverse_taper:-missing}"
printf 'HN operator heartbeat required: %s\n' "${operator_heartbeat_required:-missing}"
printf 'HN operator heartbeat timeout: %s\n' "${operator_heartbeat_timeout:-missing}"
grep -q '0.17' <<<"${yaw_tolerance}" || failed=true
grep -q 'False' <<<"${rotate_to_heading}" || failed=true
grep -q 'False' <<<"${allow_reversing}" || failed=true
grep -q '0.01' <<<"${movement_radius}" || failed=true
grep -q '120.0' <<<"${movement_allowance}" || failed=true
grep -q '5.0' <<<"${controller_failure_tolerance}" || failed=true
grep -q 'odom' <<<"${local_costmap_frame}" || failed=true
grep -q '45.0' <<<"${adapter_progress_timeout}" || failed=true
grep -q '0.03' <<<"${adapter_progress_displacement}" || failed=true
grep -q '0.52' <<<"${adapter_small_spin_max}" || failed=true
grep -Eq '3.14159|3.141592|3.141593' <<<"${adapter_alignment_max}" || failed=true
grep -q 'False' <<<"${adapter_automatic_reverse}" || failed=true
grep -Eq '3.14159|3.141592|3.141593' \
  <<<"${adapter_final_alignment_max}" || failed=true
grep -q '3.0' <<<"${adapter_reverse_allowance}" || failed=true
grep -q '0.35' <<<"${adapter_terminal_handoff}" || failed=true
grep -q '0.1' <<<"${adapter_terminal_tolerance}" || failed=true
grep -q '3.0' <<<"${adapter_terminal_allowance}" || failed=true
grep -q 'navigate_to_pose_no_spin.xml' <<<"${bt_xml}" || failed=true
# The deployed planner is deliberately Navfn A*. Automatic navigation is
# spin-then-forward and never selects /backup. The /backup behavior and its
# ultrasonic safety gate remain available for an explicit reverse operation.
grep -q 'NavfnPlanner' <<<"${planner_plugin}" || failed=true
grep -q 'drive_on_heading' <<<"${behavior_plugins}" || failed=true
grep -q 'odom' <<<"${behavior_frame}" || failed=true
grep -q '1.5' <<<"${collision_source_timeout}" || failed=true
grep -q '0.5' <<<"${slowdown_ratio}" || failed=true
grep -q '10' <<<"${slowdown_max_points}" || failed=true
grep -q 'True' <<<"${reverse_straight_only}" || failed=true
grep -q '/direct_reverse_plan' <<<"${reverse_plan_topic}" || failed=true
grep -q '3.0' <<<"${reverse_plan_timeout}" || failed=true
grep -q '0.1' <<<"${straight_lateral_limit}" || failed=true
grep -q '0.12' <<<"${straight_heading_limit}" || failed=true
grep -q '2' <<<"${straight_confirmations}" || failed=true
grep -q 'True' <<<"${allow_small_spin}" || failed=true
grep -q '3.25' <<<"${gate_small_spin_max}" || failed=true
grep -q '0.12' <<<"${gate_small_spin_rate}" || failed=true
grep -q '0.1' <<<"${gate_small_spin_reset}" || failed=true
grep -q '0.22' <<<"${sonar_stop_distance}" || failed=true
grep -q '0.35' <<<"${sonar_clear_distance}" || failed=true
grep -q '0.25' <<<"${sonar_turn_allow_distance}" || failed=true
grep -q 'True' <<<"${sonar_self_echo_enabled}" || failed=true
grep -q '0.16' <<<"${sonar_self_echo_min}" || failed=true
grep -q '0.22' <<<"${sonar_self_echo_max}" || failed=true
grep -q '0.2' <<<"${sonar_setback}" || failed=true
grep -q '0.02' <<<"${sonar_tail_clearance}" || failed=true
grep -q '1.5' <<<"${sonar_hold}" || failed=true
grep -q '3' <<<"${sonar_clear_samples}" || failed=true
grep -q '8' <<<"${sonar_no_echo_samples}" || failed=true
grep -q 'True' <<<"${sonar_turn_guard}" || failed=true
grep -q 'True' <<<"${sonar_reverse_taper}" || failed=true
grep -q 'True' <<<"${operator_heartbeat_required}" || failed=true
grep -q '2.0' <<<"${operator_heartbeat_timeout}" || failed=true
grep -q 'spin' <<<"${behavior_plugins}" || failed=true
grep -q 'wait' <<<"${behavior_plugins}" || failed=true
# BackUp and DriveOnHeading are intentionally present as bounded action
# servers. The no-spin BT does not invoke them autonomously; only the
# direction-aligned adapter calls them after its straight-corridor checks.

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
    dyaw = a[3] - expected_yaw;
    while (dyaw > 180) dyaw -= 360;
    while (dyaw < -180) dyaw += 360;
    if (dyaw < 0) dyaw = -dyaw;
    exit !(dx <= 0.002 && dy <= 0.002 && dz <= 0.002 &&
      droll <= 0.10 && dpitch <= 0.10 && dyaw <= 1.0);
  }'; then
  echo "LiDAR TF mismatch: expected xyz=(${EXPECTED_LIDAR_X_M},${EXPECTED_LIDAR_Y_M},${EXPECTED_LIDAR_Z_M}), rpy=(${EXPECTED_LIDAR_ROLL_DEG},${EXPECTED_LIDAR_PITCH_DEG},${EXPECTED_LIDAR_YAW_DEG}) deg" >&2
  failed=true
fi

check_sonar_tf() {
  local child="$1"
  local expected_y="$2"
  local output="/tmp/${child}_geometry.txt"
  local xyz rpy
  timeout 8 "${ROS_ENV}" env \
    FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
    ros2 run tf2_ros tf2_echo base_link "${child}" \
    >"${output}" 2>&1 || true
  cat "${output}"
  xyz="$(awk '
    /Translation: \[/{gsub(/[][]/, ""); gsub(/,/, ""); print $3, $4, $5; exit}
  ' "${output}")"
  rpy="$(awk '
    /RPY \(degree\) \[/{gsub(/[][]/, ""); gsub(/,/, ""); print $6, $7, $8; exit}
  ' "${output}")"
  if ! awk -v xyz="${xyz}" -v rpy="${rpy}" \
    -v expected_x="${EXPECTED_SONAR_X_M}" \
    -v expected_y="${expected_y}" \
    -v expected_z="${EXPECTED_SONAR_Z_M}" \
    -v expected_yaw="${EXPECTED_SONAR_YAW_DEG}" '
      BEGIN {
        split(xyz, p, " "); split(rpy, a, " ");
        dx = p[1] - expected_x; if (dx < 0) dx = -dx;
        dy = p[2] - expected_y; if (dy < 0) dy = -dy;
        dz = p[3] - expected_z; if (dz < 0) dz = -dz;
        dyaw = a[3] - expected_yaw;
        while (dyaw > 180) dyaw -= 360;
        while (dyaw < -180) dyaw += 360;
        if (dyaw < 0) dyaw = -dyaw;
        exit !(dx <= 0.002 && dy <= 0.002 && dz <= 0.002 &&
          dyaw <= 1.0);
      }'; then
    echo "Rear ultrasonic TF mismatch for ${child}." >&2
    return 1
  fi
}

echo "=== Rear ultrasonic mounting transforms ==="
check_sonar_tf rear_ultrasonic_left "${EXPECTED_SONAR_LEFT_Y_M}" \
  || failed=true
check_sonar_tf rear_ultrasonic_right "${EXPECTED_SONAR_RIGHT_Y_M}" \
  || failed=true

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
