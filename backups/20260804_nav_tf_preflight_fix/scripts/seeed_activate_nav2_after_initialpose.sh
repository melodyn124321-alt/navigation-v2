#!/usr/bin/env bash
set -euo pipefail

ROS_ROOT="${ROS_ROOT:-/home/seeed/ros2}"
ROS_ENV="${ROS_ENV:-${ROS_ROOT}/use_ros_env.sh}"
NAV2_PARAMS="${NAV2_PARAMS:-${ROS_ROOT}/nav2/nav2_pcd_ndt_manual_002.yaml}"
DEFAULT_NAV_YAML="${ROS_ROOT}/maps/replay/fastlio_map_manual_001_level_groundsafe_20260729_nav.yaml"
if [[ -r "${ROS_ROOT}/maps/replay/latest_raw_livox_manual_001_nav_target.txt" ]]; then
  read -r DEFAULT_NAV_YAML < "${ROS_ROOT}/maps/replay/latest_raw_livox_manual_001_nav_target.txt"
fi
NAV_YAML="${1:-${DEFAULT_NAV_YAML}}"
TF_CHECKER="${TF_CHECKER:-${ROS_ROOT}/scripts/nav_tf_readiness_check.py}"
LOCALIZATION_VERIFIER="${LOCALIZATION_VERIFIER:-${ROS_ROOT}/scripts/verify_localization_stability.py}"
NAV2_LIFECYCLE="${NAV2_LIFECYCLE:-${ROS_ROOT}/scripts/nav2_manual_lifecycle.py}"
NAV2_ACTION_CHECKER="${NAV2_ACTION_CHECKER:-${ROS_ROOT}/scripts/nav2_action_readiness_check.py}"
OBSTACLE_REPUBLISHER="${OBSTACLE_REPUBLISHER:-${ROS_ROOT}/scripts/nav_obstacle_cloud_republisher.py}"
ALIGNED_GOAL_ADAPTER="${ALIGNED_GOAL_ADAPTER:-${ROS_ROOT}/scripts/aligned_nav_goal_adapter.py}"
RVIZ_GOAL_BRIDGE="${RVIZ_GOAL_BRIDGE:-${ROS_ROOT}/scripts/rviz_goal_pose_bridge.py}"
GOAL_MARKER="${GOAL_MARKER:-${ROS_ROOT}/scripts/nav_goal_marker_from_plan.py}"
OBSTACLE_ALARM="${OBSTACLE_ALARM:-${ROS_ROOT}/scripts/nav_obstacle_block_alarm.py}"
NO_SPIN_BT="${NO_SPIN_BT:-${ROS_ROOT}/nav2/navigate_to_pose_no_spin.xml}"
NO_SPIN_THROUGH_BT="${NO_SPIN_THROUGH_BT:-${ROS_ROOT}/nav2/navigate_through_poses_no_spin.xml}"
NAV_FASTDDS_PROFILE="${NAV_FASTDDS_PROFILE:-${ROS_ROOT}/fastrtps_profile.xml}"
MAX_NDT_FITNESS="${MAX_NDT_FITNESS:-0.10}"
NODE_START_SETTLE_SEC="${NODE_START_SETTLE_SEC:-2.0}"
NODE_STOP_SETTLE_SEC="${NODE_STOP_SETTLE_SEC:-0.5}"
NODE_ACTIVATION_TIMEOUT="${NODE_ACTIVATION_TIMEOUT:-45}"
NODE_LIFECYCLE_TIMEOUT="${NODE_LIFECYCLE_TIMEOUT:-18}"
PLANNER_DISCOVERY_SETTLE_SEC="${PLANNER_DISCOVERY_SETTLE_SEC:-5.0}"
BT_DISCOVERY_SETTLE_SEC="${BT_DISCOVERY_SETTLE_SEC:-2.0}"
NODE_RETRY_SETTLE_SEC="${NODE_RETRY_SETTLE_SEC:-2.0}"
CORE_ACTION_TIMEOUT_SEC="${CORE_ACTION_TIMEOUT_SEC:-15.0}"
VELOCITY_CHAIN_RETRIES="${VELOCITY_CHAIN_RETRIES:-3}"
NAV_START_SLOW_DISTANCE_M="${NAV_START_SLOW_DISTANCE_M:-0.30}"
NAV_START_MAX_SPEED_MPS="${NAV_START_MAX_SPEED_MPS:-0.08}"
NAV_CRUISE_SPEED_MPS="${NAV_CRUISE_SPEED_MPS:-0.40}"
NAV_APPROACH_SLOW_DISTANCE_M="${NAV_APPROACH_SLOW_DISTANCE_M:-0.80}"
NAV_APPROACH_MIN_SPEED_MPS="${NAV_APPROACH_MIN_SPEED_MPS:-0.05}"
LOCALIZATION_STABILITY_SEC="${LOCALIZATION_STABILITY_SEC:-4}"
LOCALIZATION_RECOVERY_SEC="${LOCALIZATION_RECOVERY_SEC:-12}"
LOCALIZATION_REQUIRED_CONSECUTIVE="${LOCALIZATION_REQUIRED_CONSECUTIVE:-5}"
CHASSIS_LENGTH_M="${CHASSIS_LENGTH_M:-0.720}"
CHASSIS_WIDTH_M="${CHASSIS_WIDTH_M:-0.500}"
CHASSIS_HEIGHT_M="${CHASSIS_HEIGHT_M:-0.345}"
# Effective /cloud_registered_body origin height from live floor fitting.
LIDAR_HEIGHT_M="${LIDAR_HEIGHT_M:-0.749}"
LIDAR_X_M="${LIDAR_X_M:-0.100}"
LIDAR_Y_M="${LIDAR_Y_M:--0.110}"
LIDAR_Z_M="${LIDAR_Z_M:-${LIDAR_HEIGHT_M}}"
LIDAR_ROLL_RAD="${LIDAR_ROLL_RAD:--0.801751898}"
LIDAR_PITCH_RAD="${LIDAR_PITCH_RAD:--0.007094763}"
# M12 (-X) faces vehicle left (+Y): LiDAR +X faces vehicle right (-Y).
LIDAR_YAW_RAD="${LIDAR_YAW_RAD:--1.570796327}"
# Measured 2026-07-30: each rear sonar is 0.20 m forward of the rear edge,
# 0.545 m above the ground, and therefore 0.20 m above the 0.345 m chassis.
# base_link is at ground level in the chassis center.
ULTRASONIC_SENSOR_TO_REAR_EDGE_M="${ULTRASONIC_SENSOR_TO_REAR_EDGE_M:-0.20}"
ULTRASONIC_ABOVE_CHASSIS_M="${ULTRASONIC_ABOVE_CHASSIS_M:-0.20}"
ULTRASONIC_X_M="${ULTRASONIC_X_M:--0.160}"
ULTRASONIC_LEFT_Y_M="${ULTRASONIC_LEFT_Y_M:-0.18}"
ULTRASONIC_RIGHT_Y_M="${ULTRASONIC_RIGHT_Y_M:--0.18}"
ULTRASONIC_Z_M="${ULTRASONIC_Z_M:-0.545}"
ULTRASONIC_YAW_RAD="${ULTRASONIC_YAW_RAD:-3.141592654}"
ULTRASONIC_TAIL_CLEARANCE_M="${ULTRASONIC_TAIL_CLEARANCE_M:-0.02}"
ULTRASONIC_BRAKING_MARGIN_M="${ULTRASONIC_BRAKING_MARGIN_M:-0.00}"
ULTRASONIC_NOISE_MARGIN_M="${ULTRASONIC_NOISE_MARGIN_M:-0.00}"
# Compact-aisle rear control: <=0.22 m is a hard stop; >=0.35 m clears the
# latch. In between, an already-clear straight reverse is speed-tapered.
ULTRASONIC_STOP_DISTANCE_M="${ULTRASONIC_STOP_DISTANCE_M:-0.22}"
ULTRASONIC_CLEAR_DISTANCE_M="${ULTRASONIC_CLEAR_DISTANCE_M:-0.35}"
ULTRASONIC_TURN_ALLOW_DISTANCE_M="${ULTRASONIC_TURN_ALLOW_DISTANCE_M:-0.25}"
ULTRASONIC_SELF_ECHO_MIN_M="${ULTRASONIC_SELF_ECHO_MIN_M:-0.16}"
ULTRASONIC_SELF_ECHO_MAX_M="${ULTRASONIC_SELF_ECHO_MAX_M:-0.22}"
ULTRASONIC_BLOCK_HOLD_SEC="${ULTRASONIC_BLOCK_HOLD_SEC:-1.50}"
ULTRASONIC_CLEAR_SAMPLES="${ULTRASONIC_CLEAR_SAMPLES:-3}"
ULTRASONIC_NO_ECHO_CLEAR_SAMPLES="${ULTRASONIC_NO_ECHO_CLEAR_SAMPLES:-8}"

validate_chassis_lidar_geometry() {
  if ! awk -v chassis_len="${CHASSIS_LENGTH_M}" \
    -v chassis_width="${CHASSIS_WIDTH_M}" \
    -v chassis_height="${CHASSIS_HEIGHT_M}" \
    -v lidar_height="${LIDAR_HEIGHT_M}" -v lidar_z="${LIDAR_Z_M}" '
      BEGIN {
        height_error = lidar_z - lidar_height;
        if (height_error < 0) height_error = -height_error;
        exit !(chassis_len > 0 && chassis_width > 0 &&
          chassis_height > 0 && lidar_height > 0 &&
          height_error <= 0.002);
      }'; then
    echo "Invalid chassis or LiDAR geometry." >&2
    exit 1
  fi
  echo "Chassis geometry: ${CHASSIS_LENGTH_M}x${CHASSIS_WIDTH_M}x${CHASSIS_HEIGHT_M} m"
  echo "base_link -> nav_lidar: xyz=(${LIDAR_X_M},${LIDAR_Y_M},${LIDAR_Z_M})"
  echo "base_link -> nav_lidar: rpy=(${LIDAR_ROLL_RAD},${LIDAR_PITCH_RAD},${LIDAR_YAW_RAD}) rad"
  if ! awk \
    -v chassis_len="${CHASSIS_LENGTH_M}" \
    -v chassis_height="${CHASSIS_HEIGHT_M}" \
    -v setback="${ULTRASONIC_SENSOR_TO_REAR_EDGE_M}" \
    -v above="${ULTRASONIC_ABOVE_CHASSIS_M}" \
    -v sonar_x="${ULTRASONIC_X_M}" \
    -v sonar_z="${ULTRASONIC_Z_M}" \
    -v tail_clear="${ULTRASONIC_TAIL_CLEARANCE_M}" \
    -v brake="${ULTRASONIC_BRAKING_MARGIN_M}" \
    -v noise="${ULTRASONIC_NOISE_MARGIN_M}" \
    -v stop="${ULTRASONIC_STOP_DISTANCE_M}" \
    -v clear="${ULTRASONIC_CLEAR_DISTANCE_M}" '
      BEGIN {
        expected_x = -chassis_len / 2.0 + setback;
        expected_z = chassis_height + above;
        dx = sonar_x - expected_x; if (dx < 0) dx = -dx;
        dz = sonar_z - expected_z; if (dz < 0) dz = -dz;
        required_stop = setback + tail_clear + brake + noise;
        exit !(setback > 0 && above > 0 && dx <= 0.002 &&
          dz <= 0.002 && stop + 0.000001 >= required_stop &&
          clear > stop);
      }'; then
    echo "Invalid rear ultrasonic geometry or stopping margins." >&2
    exit 1
  fi
  echo "base_link -> rear sonars: x=${ULTRASONIC_X_M}m y=(${ULTRASONIC_LEFT_Y_M},${ULTRASONIC_RIGHT_Y_M})m z=${ULTRASONIC_Z_M}m yaw=${ULTRASONIC_YAW_RAD}rad"
  echo "Rear sonar compact safety: sensor_to_tail=${ULTRASONIC_SENSOR_TO_REAR_EDGE_M}m hard_stop<=${ULTRASONIC_STOP_DISTANCE_M}m reverse_full_clear>=${ULTRASONIC_CLEAR_DISTANCE_M}m turn_allow>${ULTRASONIC_TURN_ALLOW_DISTANCE_M}m minimum_tail_clearance=${ULTRASONIC_TAIL_CLEARANCE_M}m"
  echo "Rear sonar maneuver guards: left turn=both sensors, right turn=both sensors, reverse=both sensors; speed taper active between stop and clear."
  if ! awk \
    -v start_dist="${NAV_START_SLOW_DISTANCE_M}" \
    -v start_speed="${NAV_START_MAX_SPEED_MPS}" \
    -v cruise="${NAV_CRUISE_SPEED_MPS}" \
    -v approach_dist="${NAV_APPROACH_SLOW_DISTANCE_M}" \
    -v approach_min="${NAV_APPROACH_MIN_SPEED_MPS}" '
      BEGIN {
        exit !(start_dist > 0 && approach_dist > start_dist &&
          approach_min > 0 && approach_min <= start_speed &&
          start_speed < cruise);
      }'; then
    echo "Invalid three-stage navigation speed profile." >&2
    exit 1
  fi
  echo "Navigation speed profile: START<=${NAV_START_MAX_SPEED_MPS}m/s for ${NAV_START_SLOW_DISTANCE_M}m; CRUISE<=${NAV_CRUISE_SPEED_MPS}m/s; APPROACH inside ${NAV_APPROACH_SLOW_DISTANCE_M}m down to ${NAV_APPROACH_MIN_SPEED_MPS}m/s."
}

for path in "${NAV2_PARAMS}" "${NAV_YAML}" "${TF_CHECKER}" \
  "${LOCALIZATION_VERIFIER}" \
  "${NAV2_LIFECYCLE}" "${NAV2_ACTION_CHECKER}" \
  "${OBSTACLE_REPUBLISHER}" "${ALIGNED_GOAL_ADAPTER}" \
  "${RVIZ_GOAL_BRIDGE}" "${GOAL_MARKER}" "${OBSTACLE_ALARM}" \
  "${NO_SPIN_BT}" "${NO_SPIN_THROUGH_BT}" "${NAV_FASTDDS_PROFILE}"; do
  if [[ ! -r "${path}" ]]; then
    echo "Required file is missing or unreadable: ${path}" >&2
    exit 1
  fi
done

mkdir -p "${ROS_ROOT}/logs"
validate_chassis_lidar_geometry
if [[ "${CHECK_GEOMETRY_ONLY:-false}" == "true" ]]; then
  exit 0
fi

run_ros() {
  "${ROS_ENV}" env FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" "$@"
}

wait_for_typed_message() {
  local topic="$1"
  local type="$2"
  local required_pattern="${3:-}"
  local attempt
  local output="/tmp/nav_live_${topic//\//_}.txt"
  for attempt in $(seq 1 6); do
    timeout 6 "${ROS_ENV}" env \
      FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
      ros2 topic echo "${topic}" "${type}" --once \
      >"${output}" 2>&1 || true
    if [[ -s "${output}" ]] \
      && { [[ -z "${required_pattern}" ]] \
        || grep -q "${required_pattern}" "${output}"; }; then
      echo "${topic} has a live ${type} message."
      return 0
    fi
    echo "Waiting for a live message on ${topic} (${attempt}/6)..."
    sleep 1
  done
  cat "${output}" >&2 || true
  return 1
}

wait_for_lidar_tf() {
  local attempt
  for attempt in $(seq 1 8); do
    timeout 4 "${ROS_ENV}" env \
      FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
      ros2 run tf2_ros tf2_echo base_link nav_lidar \
      >/tmp/nav_lidar_tf_check.txt 2>&1 || true
    if grep -q -- '- Translation:' /tmp/nav_lidar_tf_check.txt; then
      cat /tmp/nav_lidar_tf_check.txt
      return 0
    fi
    echo "Waiting for base_link -> nav_lidar TF (${attempt}/8)..."
    sleep 1
  done
  cat /tmp/nav_lidar_tf_check.txt >&2 || true
  return 1
}

wait_for_ultrasonic_tf() {
  local frame="$1"
  local attempt
  for attempt in $(seq 1 8); do
    timeout 4 "${ROS_ENV}" env \
      FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
      ros2 run tf2_ros tf2_echo base_link "${frame}" \
      >/tmp/"${frame}"_tf_check.txt 2>&1 || true
    if grep -q -- '- Translation:' /tmp/"${frame}"_tf_check.txt; then
      return 0
    fi
    sleep 1
  done
  cat /tmp/"${frame}"_tf_check.txt >&2 || true
  return 1
}

wait_for_navigation_tf() {
  local samples="${1:-12}"
  timeout 55 "${ROS_ENV}" env FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
    python3 "${TF_CHECKER}" \
    --map-frame map --odom-frame odom --base-frame base_link \
    --samples "${samples}" --timeout 45
}

wait_for_goal_adapter() {
  local attempt nodes action_info status_info
  for attempt in $(seq 1 12); do
    nodes="$(run_ros ros2 node list 2>/dev/null || true)"
    action_info="$(run_ros ros2 action info /aligned_navigate_to_pose 2>/dev/null || true)"
    status_info="$(run_ros ros2 topic info /aligned_goal_status 2>/dev/null || true)"
    if grep -qx '/aligned_nav_goal_adapter' <<<"${nodes}" \
      && grep -Eq 'Action servers: [1-9]' <<<"${action_info}" \
      && grep -Eq 'Publisher count: [1-9]' <<<"${status_info}"; then
      echo "Goal adapter is visible as /aligned_navigate_to_pose action server."
      return 0
    fi
    echo "Waiting for goal adapter DDS discovery (${attempt}/12)..."
    sleep 2
  done
  return 1
}

wait_for_topic_connection() {
  local topic="$1"
  local min_publishers="$2"
  local min_subscribers="$3"
  local attempt info publishers subscribers
  for attempt in $(seq 1 12); do
    info="$(run_ros ros2 topic info "${topic}" 2>/dev/null || true)"
    publishers="$(awk '/Publisher count:/{print $3}' <<<"${info}")"
    subscribers="$(awk '/Subscription count:/{print $3}' <<<"${info}")"
    if (( ${publishers:-0} >= min_publishers \
      && ${subscribers:-0} >= min_subscribers )); then
      printf '%s ready: publishers=%s subscribers=%s\n' \
        "${topic}" "${publishers}" "${subscribers}"
      return 0
    fi
    sleep 2
  done
  echo "${topic} did not reach publishers>=${min_publishers}, subscribers>=${min_subscribers}." >&2
  return 1
}

stop_old_navigation() {
  local remaining forced=false
  remaining="$(ps -eo pid=,args= | awk '
    /planner_server|controller_server|smoother_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|collision_monitor|lifecycle_manager_navigation|nav_obstacle_cloud_republisher.py|nav_tf_direct_relay.py|aligned_nav_goal_adapter.py|rviz_goal_pose_bridge.py|nav_goal_marker_from_plan.py|nav_obstacle_block_alarm.py|ndt_goal_controller.py|nav_motion_safety_gate.py|static_transform_publisher.*nav_lidar|static_transform_publisher.*rear_ultrasonic/ &&
    $0 !~ /awk/ &&
    $0 !~ /seeed_activate_nav2_after_initialpose/ {print $1}
  ')"
  if [[ -n "${remaining}" ]]; then
    # shellcheck disable=SC2086
    kill ${remaining} 2>/dev/null || true
  fi
  for _ in $(seq 1 32); do
    remaining="$(ps -eo pid=,args= | awk '
      /planner_server|controller_server|smoother_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|collision_monitor|lifecycle_manager_navigation|nav_obstacle_cloud_republisher.py|nav_tf_direct_relay.py|aligned_nav_goal_adapter.py|rviz_goal_pose_bridge.py|nav_goal_marker_from_plan.py|nav_obstacle_block_alarm.py|ndt_goal_controller.py|nav_motion_safety_gate.py|static_transform_publisher.*nav_lidar|static_transform_publisher.*rear_ultrasonic/ &&
      $0 !~ /awk/ &&
      $0 !~ /seeed_activate_nav2_after_initialpose/ {print $1}
    ')"
    [[ -z "${remaining}" ]] && break
    sleep 0.25
  done
  if [[ -n "${remaining}" ]]; then
    forced=true
    echo "Forcing stale navigation PIDs after 8 seconds: ${remaining}" >&2
    # shellcheck disable=SC2086
    kill -9 ${remaining} 2>/dev/null || true
  fi
  # A forced exit cannot send a DDS participant-dispose message.  The active
  # Fast DDS profile has a 10-second lease, so let stale graph entries expire
  # once here instead of paying repeated discovery timeouts for every node.
  if [[ "${forced}" == true ]]; then
    sleep 11
  fi
  sleep 2
}

start_node() {
  local log_name="$1"
  shift
  setsid -f "${ROS_ENV}" env FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
    "$@" > "${ROS_ROOT}/logs/${log_name}.log" 2>&1 < /dev/null
  # Nodes are still started sequentially. The lifecycle service readiness check
  # below replaces the old fixed three-second registration delay.
  sleep "${NODE_START_SETTLE_SEC}"
}

stop_managed_node() {
  local node_name="$1"
  local remaining
  ps -eo pid=,args= | awk -v name="${node_name}" '
    index($0, name) &&
    $0 !~ /awk/ &&
    $0 !~ /seeed_activate_nav2_after_initialpose/ {print $1}
  ' | xargs -r kill 2>/dev/null || true
  for _ in $(seq 1 20); do
    remaining="$(ps -eo pid=,args= | awk -v name="${node_name}" '
      index($0, name) &&
      $0 !~ /awk/ &&
      $0 !~ /seeed_activate_nav2_after_initialpose/ {print $1}
    ')"
    if [[ -z "${remaining}" ]]; then
      sleep "${NODE_STOP_SETTLE_SEC}"
      return 0
    fi
    sleep 0.25
  done
  echo "/${node_name} did not exit within 5 seconds; forcing stale PIDs: ${remaining}" >&2
  # shellcheck disable=SC2086
  kill -9 ${remaining} 2>/dev/null || true
  sleep "${NODE_STOP_SETTLE_SEC}"
}

start_managed_node() {
  local node_name="$1"
  case "${node_name}" in
    controller_server)
      start_node nav2_controller_server ros2 run nav2_controller controller_server --ros-args \
        --params-file "${NAV2_PARAMS}" -r cmd_vel:=/cmd_vel_nav
      ;;
    smoother_server)
      start_node nav2_smoother_server ros2 run nav2_smoother smoother_server --ros-args \
        --params-file "${NAV2_PARAMS}"
      ;;
    planner_server)
      start_node nav2_planner_server ros2 run nav2_planner planner_server --ros-args \
        --params-file "${NAV2_PARAMS}"
      ;;
    behavior_server)
      start_node nav2_behavior_server ros2 run nav2_behaviors behavior_server --ros-args \
        --params-file "${NAV2_PARAMS}" -r cmd_vel:=/cmd_vel_nav
      ;;
    bt_navigator)
      start_node nav2_bt_navigator ros2 run nav2_bt_navigator bt_navigator --ros-args \
        --params-file "${NAV2_PARAMS}" \
        -r /goal_pose:=/bt_navigator_goal_pose_disabled \
        -r /navigate_to_pose/_action/send_goal:=/navigate_to_pose_raw/_action/send_goal \
        -r /navigate_to_pose/_action/cancel_goal:=/navigate_to_pose_raw/_action/cancel_goal \
        -r /navigate_to_pose/_action/get_result:=/navigate_to_pose_raw/_action/get_result \
        -r /navigate_to_pose/_action/feedback:=/navigate_to_pose_raw/_action/feedback \
        -r /navigate_to_pose/_action/status:=/navigate_to_pose_raw/_action/status
      ;;
    waypoint_follower)
      start_node nav2_waypoint_follower ros2 run nav2_waypoint_follower waypoint_follower \
        --ros-args --params-file "${NAV2_PARAMS}"
      ;;
    velocity_smoother)
      start_node nav2_velocity_smoother ros2 run nav2_velocity_smoother velocity_smoother \
        --ros-args --params-file "${NAV2_PARAMS}" \
        -r cmd_vel:=/cmd_vel_nav \
        -r cmd_vel_smoothed:=/cmd_vel_nav_smoothed
      ;;
    collision_monitor)
      start_node nav2_collision_monitor ros2 run nav2_collision_monitor collision_monitor \
        --ros-args --params-file "${NAV2_PARAMS}"
      ;;
    *)
      echo "Unknown managed node: ${node_name}" >&2
      return 1
      ;;
  esac
}

managed_node_process_running() {
  local node_name="$1"
  ps -eo args= | awk -v name="${node_name}" '
    index($0, name) &&
    $0 !~ /awk/ &&
    $0 !~ /seeed_activate_nav2_after_initialpose/ {found=1}
    END {exit !found}
  '
}

configure_managed_node() {
  local node_name="$1"
  echo "Configuring prestarted /${node_name}..."
  if ! timeout "${NODE_ACTIVATION_TIMEOUT}" "${ROS_ENV}" env \
    FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
    python3 "${NAV2_LIFECYCLE}" --configure-only \
      --timeout "${NODE_LIFECYCLE_TIMEOUT}" "${node_name}" \
      > "${ROS_ROOT}/logs/nav2_configure_${node_name}.log" 2>&1; then
    cat "${ROS_ROOT}/logs/nav2_configure_${node_name}.log" >&2 || true
    return 1
  fi
}

start_and_activate_managed_node() {
  local node_name="$1"
  local attempt log_file post_configure_settle
  log_file="${ROS_ROOT}/logs/nav2_${node_name}.log"
  case "${node_name}" in
    controller_server) log_file="${ROS_ROOT}/logs/nav2_controller_server.log" ;;
    smoother_server) log_file="${ROS_ROOT}/logs/nav2_smoother_server.log" ;;
    planner_server) log_file="${ROS_ROOT}/logs/nav2_planner_server.log" ;;
    behavior_server) log_file="${ROS_ROOT}/logs/nav2_behavior_server.log" ;;
    bt_navigator) log_file="${ROS_ROOT}/logs/nav2_bt_navigator.log" ;;
    waypoint_follower) log_file="${ROS_ROOT}/logs/nav2_waypoint_follower.log" ;;
    velocity_smoother) log_file="${ROS_ROOT}/logs/nav2_velocity_smoother.log" ;;
    collision_monitor) log_file="${ROS_ROOT}/logs/nav2_collision_monitor.log" ;;
  esac

  post_configure_settle=4.0
  case "${node_name}" in
    planner_server) post_configure_settle="${PLANNER_DISCOVERY_SETTLE_SEC}" ;;
    bt_navigator) post_configure_settle="${BT_DISCOVERY_SETTLE_SEC}" ;;
  esac

  for attempt in 1 2 3; do
    if (( attempt == 1 )); then
      if managed_node_process_running "${node_name}"; then
        echo "Activating prestarted /${node_name} (attempt ${attempt}/3)..."
      else
        echo "Starting /${node_name} (attempt ${attempt}/3)..."
        stop_managed_node "${node_name}"
        start_managed_node "${node_name}"
      fi
    else
      # Keep the same DDS participant alive between lifecycle retries.  Nav2
      # action discovery on this ARM host can outlive the first activation
      # timeout; restarting here used to reset discovery on every attempt.
      echo "Retrying lifecycle activation for existing /${node_name} " \
        "(attempt ${attempt}/3)..."
      sleep "${NODE_RETRY_SETTLE_SEC}"
      if ! ps -eo args= | awk -v name="${node_name}" '
        index($0, name) &&
        $0 !~ /awk/ &&
        $0 !~ /seeed_activate_nav2_after_initialpose/ {found=1}
        END {exit !found}
      '; then
        echo "/${node_name} process exited; starting it again." >&2
        start_managed_node "${node_name}"
      fi
    fi
    if timeout "${NODE_ACTIVATION_TIMEOUT}" "${ROS_ENV}" env \
      FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
      python3 "${NAV2_LIFECYCLE}" --timeout "${NODE_LIFECYCLE_TIMEOUT}" \
      --post-configure-settle "${post_configure_settle}" \
      "${node_name}" \
      > "${ROS_ROOT}/logs/nav2_lifecycle_${node_name}.log" 2>&1; then
      echo "  /${node_name}: active [3]"
      return 0
    fi
    echo "/${node_name} failed to activate on attempt ${attempt}." >&2
    tail -n 80 "${ROS_ROOT}/logs/nav2_lifecycle_${node_name}.log" >&2 || true
    tail -n 100 "${log_file}" >&2 || true
  done

  stop_managed_node "${node_name}"
  return 1
}

wait_for_core_action_servers() {
  timeout 45 "${ROS_ENV}" env \
    FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
    ROS2CLI_DISABLE_DAEMON=1 \
    python3 "${NAV2_ACTION_CHECKER}" \
      --timeout "${CORE_ACTION_TIMEOUT_SEC}" \
      --consecutive 3
}

velocity_chain_ready() {
  local topic info publishers subscribers
  for topic in /cmd_vel_nav_smoothed /cmd_vel_nav_collision_safe /cmd_vel; do
    info="$(run_ros ros2 topic info "${topic}" 2>/dev/null || true)"
    publishers="$(awk '/Publisher count:/{print $3}' <<<"${info}")"
    subscribers="$(awk '/Subscription count:/{print $3}' <<<"${info}")"
    if (( ${publishers:-0} < 1 || ${subscribers:-0} < 1 )); then
      printf '%s incomplete: publishers=%s subscribers=%s\n' \
        "${topic}" "${publishers:-0}" "${subscribers:-0}" >&2
      return 1
    fi
  done
}

ensure_velocity_chain() {
  local attempt
  for attempt in $(seq 1 "${VELOCITY_CHAIN_RETRIES}"); do
    if wait_for_topic_connection /cmd_vel_nav_smoothed 1 1 \
      && wait_for_topic_connection /cmd_vel_nav_collision_safe 1 1 \
      && wait_for_topic_connection /cmd_vel 1 1 \
      && velocity_chain_ready; then
      echo "Velocity chain endpoints are connected."
      return 0
    fi
    if (( attempt == VELOCITY_CHAIN_RETRIES )); then
      break
    fi
    echo "Velocity chain discovery is incomplete; restarting only its " \
      "smoother and collision-monitor participants (${attempt}/${VELOCITY_CHAIN_RETRIES})." >&2
    stop_managed_node collision_monitor
    stop_managed_node velocity_smoother
    if ! start_and_activate_managed_node velocity_smoother \
      || ! start_and_activate_managed_node collision_monitor; then
      return 1
    fi
  done
  return 1
}

wait_for_velocity_smoother_endpoints() {
  local attempt input_info output_info
  for attempt in $(seq 1 12); do
    input_info="$(run_ros ros2 topic info /cmd_vel_nav 2>/dev/null || true)"
    output_info="$(run_ros ros2 topic info /cmd_vel_nav_smoothed 2>/dev/null || true)"
    if grep -q 'Subscription count: [1-9]' <<<"${input_info}" \
      && grep -q 'Publisher count: [1-9]' <<<"${output_info}"; then
      echo "Velocity smoother endpoints are visible."
      return 0
    fi
    sleep 1
  done
  echo "Velocity smoother process is active but its DDS endpoints are incomplete." >&2
  return 1
}

start_velocity_smoother_with_endpoints() {
  local endpoint_attempt
  for endpoint_attempt in 1 2 3; do
    if start_and_activate_managed_node velocity_smoother \
      && wait_for_velocity_smoother_endpoints; then
      return 0
    fi
    echo "Restarting /velocity_smoother for endpoint discovery " \
      "(${endpoint_attempt}/3)." >&2
    stop_managed_node velocity_smoother
  done
  return 1
}

motion_gate_visible() {
  local info service_type
  info="$(run_ros ros2 topic info /cmd_vel 2>/dev/null || true)"
  service_type="$(run_ros ros2 service type /set_nav_motion_enabled 2>/dev/null || true)"
  grep -q 'Publisher count: [1-9]' <<<"${info}" \
    && [[ "${service_type}" == "std_srvs/srv/SetBool" ]]
}

start_motion_gate() {
  local attempt
  for attempt in 1 2; do
    start_node nav_motion_safety_gate python3 \
      "${ROS_ROOT}/scripts/nav_motion_safety_gate.py" --ros-args \
      -p input_topic:=/cmd_vel_nav_collision_safe \
      -p pre_collision_topic:=/cmd_vel_nav_smoothed \
      -p output_topic:=/cmd_vel \
      -p localization_timeout:=10.0 \
      -p odom_timeout:=0.30 \
      -p chassis_timeout:=0.50 \
      -p max_linear:="${NAV_CRUISE_SPEED_MPS}" \
      -p max_angular:=0.18 \
      -p speed_profile_enabled:=true \
      -p start_slow_distance:="${NAV_START_SLOW_DISTANCE_M}" \
      -p start_max_linear:="${NAV_START_MAX_SPEED_MPS}" \
      -p approach_slow_distance:="${NAV_APPROACH_SLOW_DISTANCE_M}" \
      -p approach_min_linear:="${NAV_APPROACH_MIN_SPEED_MPS}" \
      -p speed_profile_plan_topic:=/plan \
      -p minimum_linear_for_turn:=0.012 \
      -p max_motion_curvature:=2.25 \
      -p curvature_slack:=0.015 \
      -p allow_small_in_place_rotation:=true \
      -p max_in_place_rotation:=3.25 \
      -p max_in_place_angular:=0.12 \
      -p small_spin_reset_distance:=0.10 \
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
      -p ultrasonic_stop_distance:="${ULTRASONIC_STOP_DISTANCE_M}" \
      -p ultrasonic_clear_distance:="${ULTRASONIC_CLEAR_DISTANCE_M}" \
      -p ultrasonic_turn_allow_distance:="${ULTRASONIC_TURN_ALLOW_DISTANCE_M}" \
      -p ultrasonic_self_echo_enabled:=true \
      -p ultrasonic_self_echo_min_distance:="${ULTRASONIC_SELF_ECHO_MIN_M}" \
      -p ultrasonic_self_echo_max_distance:="${ULTRASONIC_SELF_ECHO_MAX_M}" \
      -p ultrasonic_sensor_to_rear_edge:="${ULTRASONIC_SENSOR_TO_REAR_EDGE_M}" \
      -p ultrasonic_required_tail_clearance:="${ULTRASONIC_TAIL_CLEARANCE_M}" \
      -p ultrasonic_braking_margin:="${ULTRASONIC_BRAKING_MARGIN_M}" \
      -p ultrasonic_noise_margin:="${ULTRASONIC_NOISE_MARGIN_M}" \
      -p ultrasonic_block_hold_sec:="${ULTRASONIC_BLOCK_HOLD_SEC}" \
      -p ultrasonic_clear_samples_required:="${ULTRASONIC_CLEAR_SAMPLES}" \
      -p ultrasonic_no_echo_clear_samples_required:="${ULTRASONIC_NO_ECHO_CLEAR_SAMPLES}" \
      -p ultrasonic_turn_guard_enabled:=true \
      -p ultrasonic_reverse_speed_taper_enabled:=true \
      -p operator_heartbeat_required:=true \
      -p operator_heartbeat_timeout:=2.0 \
      -p goal_lease_required:=true \
      -p goal_lease_timeout:=0.75
    for _ in $(seq 1 10); do
      if motion_gate_visible; then
        echo "Motion gate is visible in the ROS graph (attempt ${attempt})."
        return 0
      fi
      sleep 1
    done
    echo "Motion gate process was not registered; restarting once." >&2
    ps -eo pid=,args= | awk '
      /nav_motion_safety_gate.py/ && $0 !~ /awk/ {print $1}
    ' | xargs -r kill 2>/dev/null || true
    sleep 3
  done
  return 1
}

echo "[1/11] Checking live NDT localization..."
if ! timeout 25 "${ROS_ENV}" env FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
  ros2 topic echo /relocalization_odom nav_msgs/msg/Odometry --once \
    --qos-reliability best_effort \
  >/tmp/nav2_relocalization_check.txt 2>&1; then
  cat /tmp/nav2_relocalization_check.txt >&2 || true
  echo "No live /relocalization_odom. Use hn RViz 2D Pose Estimate and wait for LOCALIZED first." >&2
  exit 1
fi
ndt_fitness="$(awk '/covariance:/{found=1; next} found && /^  - /{print $2; exit}' \
  /tmp/nav2_relocalization_check.txt)"
if [[ -z "${ndt_fitness}" ]] \
  || ! awk -v value="${ndt_fitness}" -v limit="${MAX_NDT_FITNESS}" \
    'BEGIN {exit !(value <= limit)}'; then
  echo "NDT fitness=${ndt_fitness:-missing} exceeds ${MAX_NDT_FITNESS}." >&2
  echo "Set an accurate 2D Pose Estimate in hn RViz before starting Nav2." >&2
  exit 1
fi
echo "NDT fitness=${ndt_fitness} is within the ${MAX_NDT_FITNESS} limit."
echo "Checking persistent localization marker and consecutive healthy samples..."
if ! timeout 25 "${ROS_ENV}" env FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
  python3 "${LOCALIZATION_VERIFIER}" --duration "${LOCALIZATION_STABILITY_SEC}" \
  --recovery-timeout "${LOCALIZATION_RECOVERY_SEC}" \
  --required-consecutive "${LOCALIZATION_REQUIRED_CONSECUTIVE}" \
  --max-fitness "${MAX_NDT_FITNESS}"; then
  echo "Localization stability verification failed. Nav2 remains stopped." >&2
  echo "Reset 2D Pose Estimate in hn RViz and wait for LOCALIZED before retrying." >&2
  exit 1
fi

echo "[2/11] Checking stable map -> odom -> base_link TF..."
if ! wait_for_navigation_tf; then
  echo "Navigation TF is not stable. Nav2 remains stopped." >&2
  exit 1
fi

echo "[3/11] Stopping the old navigation-only processes..."
stop_old_navigation

echo "[4/11] Starting the DISARMED final motion gate first..."
echo "Starting rear ultrasonic transforms and checking live range topics..."
start_node rear_ultrasonic_left_tf ros2 run tf2_ros static_transform_publisher \
  --x "${ULTRASONIC_X_M}" --y "${ULTRASONIC_LEFT_Y_M}" \
  --z "${ULTRASONIC_Z_M}" --yaw "${ULTRASONIC_YAW_RAD}" \
  --pitch 0.0 --roll 0.0 \
  --frame-id base_link --child-frame-id rear_ultrasonic_left
start_node rear_ultrasonic_right_tf ros2 run tf2_ros static_transform_publisher \
  --x "${ULTRASONIC_X_M}" --y "${ULTRASONIC_RIGHT_Y_M}" \
  --z "${ULTRASONIC_Z_M}" --yaw "${ULTRASONIC_YAW_RAD}" \
  --pitch 0.0 --roll 0.0 \
  --frame-id base_link --child-frame-id rear_ultrasonic_right
if ! wait_for_ultrasonic_tf rear_ultrasonic_left \
  || ! wait_for_ultrasonic_tf rear_ultrasonic_right; then
  echo "Required rear ultrasonic TF is unavailable; Nav2 remains stopped." >&2
  exit 1
fi
wait_for_typed_message /ultrasonic/sensor_1/range sensor_msgs/msg/Range
wait_for_typed_message /ultrasonic/sensor_2/range sensor_msgs/msg/Range
wait_for_typed_message /ultrasonic/min_range std_msgs/msg/Float32
wait_for_typed_message /ultrasonic/healthy std_msgs/msg/Bool 'data: true'
if ! start_motion_gate; then
  echo "Motion safety gate did not join the ROS graph; Nav2 remains stopped." >&2
  exit 1
fi

echo "[5/11] Starting the chassis-oriented dynamic obstacle cloud..."
start_node nav_lidar_static_tf ros2 run tf2_ros static_transform_publisher \
  --x "${LIDAR_X_M}" --y "${LIDAR_Y_M}" --z "${LIDAR_Z_M}" \
  --yaw "${LIDAR_YAW_RAD}" --pitch "${LIDAR_PITCH_RAD}" \
  --roll "${LIDAR_ROLL_RAD}" \
  --frame-id base_link --child-frame-id nav_lidar
if ! wait_for_lidar_tf; then
  echo "Required base_link -> nav_lidar TF is unavailable." >&2
  exit 1
fi
# Keep thin poles, chair legs, and box edges while retaining bounded CPU use.
start_node nav_obstacle_cloud_republisher python3 "${OBSTACLE_REPUBLISHER}" --ros-args \
  -p output_frame:=base_link \
  -p lidar_x_m:="${LIDAR_X_M}" -p lidar_y_m:="${LIDAR_Y_M}" \
  -p lidar_z_m:="${LIDAR_Z_M}" -p lidar_roll_rad:="${LIDAR_ROLL_RAD}" \
  -p lidar_pitch_rad:="${LIDAR_PITCH_RAD}" \
  -p lidar_yaw_rad:="${LIDAR_YAW_RAD}" \
  -p output_stamp_delay_sec:=0.15 \
  -p self_min_x:=-0.38 -p self_max_x:=0.38 -p self_half_width:=0.27 \
  -p point_stride:=6

obstacle_ready=false
for attempt in $(seq 1 12); do
  timeout 5 "${ROS_ENV}" env FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
    ros2 topic echo /nav_obstacle_cloud_status std_msgs/msg/String --once \
    >/tmp/nav_obstacle_cloud_status.txt 2>&1 || true
  obstacle_age="$(sed -n 's/.*age=\([0-9.]*\)s.*/\1/p' \
    /tmp/nav_obstacle_cloud_status.txt | head -n 1)"
  if grep -q "ok received=" /tmp/nav_obstacle_cloud_status.txt \
    && [[ -n "${obstacle_age}" ]] \
    && awk -v age="${obstacle_age}" 'BEGIN {exit !(age <= 1.00)}'; then
    obstacle_ready=true
    break
  fi
  echo "Waiting for dynamic obstacle cloud (${attempt}/12)..."
  sleep 2
done
cat /tmp/nav_obstacle_cloud_status.txt || true
if [[ "${obstacle_ready}" != true ]]; then
  echo "Dynamic obstacle cloud is missing or older than 1.00 s." >&2
  exit 1
fi

echo "[6/11] Starting and activating map_server..."
stop_managed_node map_server
start_node nav2_map_server ros2 run nav2_map_server map_server --ros-args \
  -p use_sim_time:=false \
  -p yaml_filename:="${NAV_YAML}" \
  -p topic_name:=/map \
  -p frame_id:=map
if ! timeout "${NODE_ACTIVATION_TIMEOUT}" "${ROS_ENV}" env \
  FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
  python3 "${NAV2_LIFECYCLE}" --timeout "${NODE_LIFECYCLE_TIMEOUT}" map_server \
  > "${ROS_ROOT}/logs/nav2_lifecycle_map_server.log" 2>&1; then
  cat "${ROS_ROOT}/logs/nav2_lifecycle_map_server.log" >&2 || true
  tail -n 120 "${ROS_ROOT}/logs/nav2_map_server.log" >&2 || true
  echo "map_server failed to configure and activate." >&2
  exit 1
fi
echo "  /map_server: active [3]"

managed_nodes=(
  planner_server controller_server smoother_server behavior_server
  bt_navigator waypoint_follower velocity_smoother collision_monitor
)

echo "[7/11] Starting and activating Nav2 nodes one at a time..."
# Do not start every Nav2 participant concurrently on this seeed host.  Live
# testing showed that a burst of eight Fast DDS participants can prevent the
# controller's private TF buffer from ever discovering /tf.  Sequential launch
# is slower on a cold start but deterministic; the daily warm path does not
# repeat this step.
for node in controller_server smoother_server planner_server behavior_server; do
  if ! start_and_activate_managed_node "${node}"; then
    echo "Failed to start /${node} after three attempts. Motion remains DISARMED." >&2
    exit 1
  fi
done

if ! wait_for_core_action_servers; then
  echo "Controller/planner action servers are incomplete. Motion remains DISARMED." >&2
  exit 1
fi

for node in bt_navigator waypoint_follower; do
  if ! start_and_activate_managed_node "${node}"; then
    echo "Failed to start /${node} after three attempts. Motion remains DISARMED." >&2
    exit 1
  fi
done
if ! start_velocity_smoother_with_endpoints; then
  echo "Failed to start /velocity_smoother with complete DDS endpoints." >&2
  echo "Motion remains DISARMED." >&2
  exit 1
fi
if ! start_and_activate_managed_node collision_monitor; then
  echo "Failed to start /collision_monitor after three attempts. Motion remains DISARMED." >&2
  exit 1
fi

echo "[8/11] All managed Nav2 nodes reached active [3]."

echo "[9/11] Starting the persistent Nav2 goal marker..."
goal_adapter_ready=false
for attempt in 1 2 3; do
  stop_managed_node aligned_nav_goal_adapter
  start_node aligned_nav_goal_adapter python3 "${ALIGNED_GOAL_ADAPTER}" --ros-args \
    -p approach_distance:=0.70 -p min_approach_distance:=0.45 \
    -p outer_action_name:=/aligned_navigate_to_pose \
    -p direct_reverse_plan_topic:=/direct_reverse_plan \
    -p approach_step:=0.05 -p maximum_cost:=90 -p feedback_period:=0.20 \
    -p progress_timeout:=45.0 -p progress_min_displacement:=0.03 \
    -p small_spin_min_angle:=0.08 -p small_spin_max_angle:=0.52 \
    -p direct_alignment_max_angle:=3.141593 \
    -p automatic_reverse_enabled:=false \
    -p final_alignment_max_angle:=3.141593 \
    -p small_spin_timeout:=12.0 \
    -p small_spin_effective_min_angular_speed:=0.020 \
    -p small_spin_timeout_factor:=1.50 \
    -p small_spin_timeout_margin:=5.0 \
    -p use_map_yaw_spin:=true \
    -p map_spin_command_topic:=/cmd_vel_nav \
    -p map_spin_angular_speed:=0.18 \
    -p map_spin_yaw_tolerance:=0.08 \
    -p map_spin_max_position_drift:=0.15 \
    -p straight_reverse_speed:=0.06 \
    -p reverse_time_allowance_factor:=3.0 \
    -p terminal_handoff_distance:=0.35 \
    -p terminal_position_tolerance:=0.10 \
    -p terminal_forward_speed:=0.04 \
    -p terminal_time_allowance_factor:=3.0 \
    -p terminal_alignment_max_angle:=0.52 \
    -p policy_block_timeout:=6.0 \
    -p stale_command_abort_timeout:=15.0 \
    -p collision_block_abort_timeout:=3.0
  if wait_for_goal_adapter; then
    goal_adapter_ready=true
    break
  fi
  echo "Goal adapter failed DDS discovery on attempt ${attempt}/3; restarting it." >&2
done
if [[ "${goal_adapter_ready}" != true ]]; then
  echo "Goal adapter is unavailable; motion remains DISARMED." >&2
  exit 1
fi
start_node rviz_goal_pose_bridge python3 "${RVIZ_GOAL_BRIDGE}" --ros-args \
  -p goal_topic:=/goal_pose \
  -p action_name:=/aligned_navigate_to_pose \
  -p required_frame:=map \
  -p duplicate_window:=3.0
wait_for_topic_connection /goal_pose 0 1
goal_pose_endpoints="$(
  timeout 15 "${ROS_ENV}" env \
    FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
    ROS2CLI_DISABLE_DAEMON=1 \
    ros2 topic info /goal_pose -v 2>/dev/null || true
)"
if ! grep -q '^Subscription count: 1$' <<<"${goal_pose_endpoints}" \
  || ! grep -q '^Node name: rviz_goal_pose_bridge$' \
    <<<"${goal_pose_endpoints}" \
  || grep -q '^Node name: bt_navigator$' <<<"${goal_pose_endpoints}"; then
  echo "/goal_pose is not isolated to rviz_goal_pose_bridge." >&2
  printf '%s\n' "${goal_pose_endpoints}" >&2
  echo "Motion remains DISARMED to prevent duplicate raw/aligned goals." >&2
  exit 1
fi
echo "/goal_pose is isolated to rviz_goal_pose_bridge."
wait_for_topic_connection /rviz_goal_pose_bridge_status 1 0
start_node nav_goal_marker python3 "${GOAL_MARKER}" --ros-args \
  -p goal_tolerance:=0.08 -p text_offset:=0.70 -p pulse_period:=0.40

echo "[10/11] Starting the 60-second persistent-obstacle alarm..."
start_node nav_obstacle_block_alarm python3 "${OBSTACLE_ALARM}" --ros-args \
  -p stop_radius:=0.60 -p min_height:=0.08 -p max_height:=1.60 \
  -p min_points:=3 -p alarm_timeout_sec:=60.0 -p clear_hold_sec:=0.50

wait_for_topic_connection /nav_goal_markers 1 0
wait_for_topic_connection /nav_obstacle_alarm 1 0
wait_for_topic_connection /nav_obstacle_alarm_markers 1 0
if ! ensure_velocity_chain; then
  echo "Velocity chain endpoints are incomplete after ${VELOCITY_CHAIN_RETRIES} attempts." >&2
  echo "Motion remains DISARMED." >&2
  exit 1
fi

echo "[11/11] Verifying lifecycle, obstacle layers, goal marker, alarm, and velocity isolation..."
if ! timeout 45 "${ROS_ENV}" env \
  FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
  python3 "${NAV2_LIFECYCLE}" --check-only --timeout 8 \
  map_server "${managed_nodes[@]}"; then
  echo "At least one Nav2 lifecycle node is not active." >&2
  exit 1
fi

if ! wait_for_navigation_tf 5; then
  echo "TF became unstable after Nav2 activation." >&2
  exit 1
fi
if ! wait_for_core_action_servers; then
  echo "Controller/planner action servers disappeared after activation." >&2
  exit 1
fi

# Topic discovery and the republisher's own status do not prove that the
# costmap's TF message filter actually accepted a cloud.  A few warnings during
# the first DDS/TF handshake are harmless once they stop.  Fail only while the
# warning is still current and therefore represents an unrecovered buffer.
last_costmap_warning_sec="$(sed -n \
  's/.*\[\([0-9][0-9]*\)\.[0-9]*\].*observation buffer has not been updated.*/\1/p' \
  "${ROS_ROOT}/logs/nav2_controller_server.log" | tail -n 1)"
now_sec="$(date +%s)"
if [[ -n "${last_costmap_warning_sec}" ]] \
  && (( now_sec - last_costmap_warning_sec <= 3 )); then
  echo "Local costmap rejected /nav_obstacle_cloud; observation buffer is stale." >&2
  echo "Motion remains DISARMED. Check cloud stamps and odom/map TF." >&2
  exit 1
fi
if [[ -n "${last_costmap_warning_sec}" ]]; then
  echo "Local costmap recovered from its startup observation delay " \
    "$((now_sec - last_costmap_warning_sec)) seconds ago."
fi

for topic in /nav_obstacle_cloud /local_costmap/costmap \
  /global_costmap/costmap /goal_pose /rviz_goal_pose_bridge_status \
  /nav_goal_markers /nav_obstacle_alarm \
  /nav_obstacle_alarm_markers /cmd_vel_nav_collision_safe /cmd_vel; do
  echo "--- ${topic}"
  timeout 10 "${ROS_ENV}" env FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
    ros2 topic info "${topic}" || true
done

echo "--- dynamic obstacle status"
final_obstacle_status="$(timeout 8 "${ROS_ENV}" env \
  FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
  ros2 topic echo /nav_obstacle_cloud_status std_msgs/msg/String --once \
    2>/dev/null || true)"
printf '%s\n' "${final_obstacle_status:-no status}"
final_obstacle_age="$(sed -n 's/.*age=\([0-9.]*\)s.*/\1/p' \
  <<<"${final_obstacle_status}" | head -n 1)"
if [[ -z "${final_obstacle_age}" ]] \
  || ! awk -v age="${final_obstacle_age}" 'BEGIN {exit !(age <= 1.00)}'; then
  echo "Dynamic obstacle cloud is stale after Nav2 activation: " \
    "age=${final_obstacle_age:-missing}s." >&2
  exit 1
fi
echo "--- motion gate status"
timeout 8 "${ROS_ENV}" env FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
  ros2 topic echo /nav_motion_status std_msgs/msg/String --once || true
echo "--- persistent obstacle alarm status"
timeout 8 "${ROS_ENV}" env FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
  ros2 topic echo /nav_obstacle_alarm_status std_msgs/msg/String --once || true

echo
echo "Nav2 dynamic avoidance is ACTIVE, but physical motion remains DISARMED."
echo "Velocity chain: Nav2 -> smoother -> collision monitor -> safety gate -> /cmd_vel"
echo "Only arm after checking the local/global costmaps in hn RViz."
