#!/usr/bin/env bash
set -euo pipefail

ROS_ROOT="${ROS_ROOT:-/home/seeed/ros2}"
ROS_ENV="${ROS_ENV:-${ROS_ROOT}/use_ros_env.sh}"
PROFILE="${NAV_FASTDDS_PROFILE:-${ROS_ROOT}/fastrtps_profile.xml}"
GATE="${ROS_ROOT}/scripts/nav_motion_safety_gate.py"
LOG="${ROS_ROOT}/logs/nav_motion_safety_gate.log"

"${ROS_ROOT}/scripts/seeed_disarm_nav_motion.sh"

gate_pids="$(
  ps -eo pid=,comm=,args= | awk '
    $2 == "python3" && /nav_motion_safety_gate.py/ && $0 !~ /awk/ {print $1}
  '
)"
if [[ -n "${gate_pids}" ]]; then
  # shellcheck disable=SC2086
  kill ${gate_pids} 2>/dev/null || true
fi
for _ in $(seq 1 20); do
  remaining="$(
    ps -eo pid=,comm=,args= | awk '
      $2 == "python3" && /nav_motion_safety_gate.py/ && $0 !~ /awk/ {print $1}
    '
  )"
  if [[ -z "${remaining}" ]]; then
    break
  fi
  sleep 0.25
done
if [[ -n "${remaining:-}" ]]; then
  echo "Old navigation motion safety gate did not stop: ${remaining}" >&2
  exit 1
fi

setsid -f "${ROS_ENV}" env \
  FASTRTPS_DEFAULT_PROFILES_FILE="${PROFILE}" \
  python3 "${GATE}" --ros-args \
  -p input_topic:=/cmd_vel_nav_collision_safe \
  -p pre_collision_topic:=/cmd_vel_nav_smoothed \
  -p output_topic:=/cmd_vel \
  -p max_fitness:="${MAX_NDT_FITNESS:-0.18}" \
  -p fitness_block_threshold:="${NDT_RUNTIME_BLOCK_FITNESS:-0.22}" \
  -p fitness_clear_threshold:="${NDT_RUNTIME_CLEAR_FITNESS:-0.18}" \
  -p fitness_bad_hold_sec:="${NDT_RUNTIME_BAD_HOLD_SEC:-2.0}" \
  -p fitness_clear_samples_required:="${NDT_RUNTIME_CLEAR_SAMPLES:-3}" \
  -p terminal_fitness_distance:="${NDT_TERMINAL_DISTANCE_M:-0.20}" \
  -p terminal_fitness_block_threshold:="${NDT_TERMINAL_BLOCK_FITNESS:-0.35}" \
  -p terminal_fitness_clear_threshold:="${NDT_TERMINAL_CLEAR_FITNESS:-0.32}" \
  -p degraded_fitness_recovery_enabled:=true \
  -p degraded_fitness_max_travel:=0.50 \
  -p degraded_fitness_max_linear:=0.08 \
  -p degraded_fitness_max_angular:=0.08 \
  -p degraded_fitness_max_rotation:=0.785398163 \
  -p degraded_fitness_clear_samples_required:=2 \
  -p localization_timeout:=10.0 \
  -p odom_timeout:=0.30 \
  -p chassis_timeout:=1.00 \
  -p max_linear:="${NAV_CRUISE_SPEED_MPS:-0.40}" \
  -p max_angular:=0.18 \
  -p speed_profile_enabled:=true \
  -p start_slow_distance:="${NAV_START_SLOW_DISTANCE_M:-0.30}" \
  -p start_max_linear:="${NAV_START_MAX_SPEED_MPS:-0.08}" \
  -p approach_slow_distance:="${NAV_APPROACH_SLOW_DISTANCE_M:-0.80}" \
  -p approach_min_linear:="${NAV_APPROACH_MIN_SPEED_MPS:-0.05}" \
  -p speed_profile_plan_topic:=/plan \
  -p minimum_linear_for_turn:=0.012 \
  -p max_motion_curvature:=2.25 \
  -p curvature_slack:=0.015 \
  -p allow_small_in_place_rotation:=true \
  -p max_in_place_rotation:=5.24 \
  -p max_in_place_angular:=0.12 \
  -p small_spin_reset_distance:=0.05 \
  -p reverse_straight_only:=true \
  -p max_reverse_angular:=0.015 \
  -p plan_topic:=/direct_reverse_plan \
  -p plan_timeout:=3.0 \
  -p straight_path_min_length:=0.05 \
  -p straight_path_max_lateral_error:=0.10 \
  -p straight_path_max_heading_span:=0.12 \
  -p straight_path_max_length_ratio:=1.03 \
  -p straight_path_confirmations_required:=2 \
  -p ultrasonic_enabled:=true \
  -p ultrasonic_motion_direction:=reverse \
  -p ultrasonic_timeout:=0.75 \
  -p ultrasonic_stop_distance:="${ULTRASONIC_STOP_DISTANCE_M:-0.22}" \
  -p ultrasonic_clear_distance:="${ULTRASONIC_CLEAR_DISTANCE_M:-0.35}" \
  -p ultrasonic_turn_allow_distance:="${ULTRASONIC_TURN_ALLOW_DISTANCE_M:-0.25}" \
  -p ultrasonic_self_echo_enabled:=true \
  -p ultrasonic_self_echo_min_distance:="${ULTRASONIC_SELF_ECHO_MIN_M:-0.08}" \
  -p ultrasonic_self_echo_max_distance:="${ULTRASONIC_SELF_ECHO_MAX_M:-0.12}" \
  -p ultrasonic_sensor_to_rear_edge:="${ULTRASONIC_SENSOR_TO_REAR_EDGE_M:-0.15}" \
  -p ultrasonic_required_tail_clearance:="${ULTRASONIC_TAIL_CLEARANCE_M:-0.02}" \
  -p ultrasonic_braking_margin:="${ULTRASONIC_BRAKING_MARGIN_M:-0.00}" \
  -p ultrasonic_noise_margin:="${ULTRASONIC_NOISE_MARGIN_M:-0.00}" \
  -p ultrasonic_block_hold_sec:="${ULTRASONIC_BLOCK_HOLD_SEC:-1.50}" \
  -p ultrasonic_clear_samples_required:="${ULTRASONIC_CLEAR_SAMPLES:-3}" \
  -p ultrasonic_no_echo_clear_samples_required:="${ULTRASONIC_NO_ECHO_CLEAR_SAMPLES:-8}" \
  -p ultrasonic_turn_guard_enabled:=true \
  -p ultrasonic_reverse_speed_taper_enabled:=true \
  -p operator_heartbeat_required:=true \
  -p operator_heartbeat_timeout:=3.0 \
  -p goal_lease_required:=true \
  -p goal_lease_timeout:=0.75 \
  >"${LOG}" 2>&1 < /dev/null

for attempt in $(seq 1 20); do
  service_type="$(
    timeout 8 "${ROS_ENV}" env \
      FASTRTPS_DEFAULT_PROFILES_FILE="${PROFILE}" \
      ROS2CLI_DISABLE_DAEMON=1 \
      ros2 service type /set_nav_motion_enabled 2>/dev/null || true
  )"
  cmd_vel_info="$(
    timeout 8 "${ROS_ENV}" env \
      FASTRTPS_DEFAULT_PROFILES_FILE="${PROFILE}" \
      ROS2CLI_DISABLE_DAEMON=1 \
      ros2 topic info /cmd_vel 2>/dev/null || true
  )"
  if [[ "${service_type}" == "std_srvs/srv/SetBool" ]] \
    && grep -q 'Publisher count: [1-9]' <<<"${cmd_vel_info}"; then
    echo "SEEED_NAV_MOTION_SAFETY_GATE_READY attempt=${attempt}"
    echo "Motion remains DISARMED."
    exit 0
  fi
  sleep 1
done

echo "Navigation motion safety gate failed to become ready." >&2
tail -100 "${LOG}" >&2 || true
exit 1
