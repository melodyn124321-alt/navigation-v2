#!/usr/bin/env bash
set -euo pipefail

ROS_ROOT="${ROS_ROOT:-/home/seeed/ros2}"
ROS_ENV="${ROS_ENV:-${ROS_ROOT}/use_ros_env.sh}"
NAV2_PARAMS="${NAV2_PARAMS:-${ROS_ROOT}/nav2/nav2_pcd_ndt_manual_002.yaml}"
NAV_YAML="${1:-${ROS_ROOT}/maps/replay/fastlio_map_manual_001_nav_current.yaml}"
TF_CHECKER="${TF_CHECKER:-${ROS_ROOT}/scripts/nav_tf_readiness_check.py}"
LOCALIZATION_VERIFIER="${LOCALIZATION_VERIFIER:-${ROS_ROOT}/scripts/verify_localization_stability.py}"
NAV2_LIFECYCLE="${NAV2_LIFECYCLE:-${ROS_ROOT}/scripts/nav2_manual_lifecycle.py}"
OBSTACLE_REPUBLISHER="${OBSTACLE_REPUBLISHER:-${ROS_ROOT}/scripts/nav_obstacle_cloud_republisher.py}"
ALIGNED_GOAL_ADAPTER="${ALIGNED_GOAL_ADAPTER:-${ROS_ROOT}/scripts/aligned_nav_goal_adapter.py}"
GOAL_MARKER="${GOAL_MARKER:-${ROS_ROOT}/scripts/nav_goal_marker_from_plan.py}"
OBSTACLE_ALARM="${OBSTACLE_ALARM:-${ROS_ROOT}/scripts/nav_obstacle_block_alarm.py}"
NO_SPIN_BT="${NO_SPIN_BT:-${ROS_ROOT}/nav2/navigate_to_pose_no_spin.xml}"
NO_SPIN_THROUGH_BT="${NO_SPIN_THROUGH_BT:-${ROS_ROOT}/nav2/navigate_through_poses_no_spin.xml}"
NAV_FASTDDS_PROFILE="${NAV_FASTDDS_PROFILE:-${ROS_ROOT}/fastrtps_profile.xml}"
MAX_NDT_FITNESS="${MAX_NDT_FITNESS:-0.10}"
NODE_START_SETTLE_SEC="${NODE_START_SETTLE_SEC:-2.0}"
NODE_STOP_SETTLE_SEC="${NODE_STOP_SETTLE_SEC:-0.5}"
NODE_ACTIVATION_TIMEOUT="${NODE_ACTIVATION_TIMEOUT:-105}"
NODE_LIFECYCLE_TIMEOUT="${NODE_LIFECYCLE_TIMEOUT:-80}"
PLANNER_DISCOVERY_SETTLE_SEC="${PLANNER_DISCOVERY_SETTLE_SEC:-20.0}"
BT_DISCOVERY_SETTLE_SEC="${BT_DISCOVERY_SETTLE_SEC:-20.0}"
NODE_RETRY_SETTLE_SEC="${NODE_RETRY_SETTLE_SEC:-10.0}"
LOCALIZATION_STABILITY_SEC="${LOCALIZATION_STABILITY_SEC:-8}"
CHASSIS_LENGTH_M="${CHASSIS_LENGTH_M:-0.720}"
CHASSIS_WIDTH_M="${CHASSIS_WIDTH_M:-0.500}"
CHASSIS_HEIGHT_M="${CHASSIS_HEIGHT_M:-0.345}"
CABINET_LENGTH_M="${CABINET_LENGTH_M:-0.480}"
CABINET_WIDTH_M="${CABINET_WIDTH_M:-0.360}"
CABINET_FRONT_GAP_M="${CABINET_FRONT_GAP_M:-0.050}"
LIDAR_FROM_REAR_M="${LIDAR_FROM_REAR_M:-1.250}"
LIDAR_HEIGHT_M="${LIDAR_HEIGHT_M:-0.700}"
LIDAR_X_M="${LIDAR_X_M:-0.890}"
LIDAR_Y_M="${LIDAR_Y_M:--0.050}"
LIDAR_Z_M="${LIDAR_Z_M:-${LIDAR_HEIGHT_M}}"
LIDAR_ROLL_RAD="${LIDAR_ROLL_RAD:--0.00005236}"
LIDAR_PITCH_RAD="${LIDAR_PITCH_RAD:-0.03940953}"
LIDAR_YAW_RAD="${LIDAR_YAW_RAD:-3.141592653589793}"

validate_vehicle_geometry() {
  if ! awk -v chassis_len="${CHASSIS_LENGTH_M}" \
    -v cabinet_len="${CABINET_LENGTH_M}" \
    -v cabinet_width="${CABINET_WIDTH_M}" \
    -v gap="${CABINET_FRONT_GAP_M}" \
    -v rear="${LIDAR_FROM_REAR_M}" -v lidar_x="${LIDAR_X_M}" \
    -v lidar_y="${LIDAR_Y_M}" \
    -v lidar_height="${LIDAR_HEIGHT_M}" -v lidar_z="${LIDAR_Z_M}" '
      BEGIN {
        expected_x = chassis_len / 2.0 + gap + cabinet_len;
        x_error = lidar_x - expected_x; if (x_error < 0) x_error = -x_error;
        rear_error = rear - (chassis_len + gap + cabinet_len);
        if (rear_error < 0) rear_error = -rear_error;
        height_error = lidar_z - lidar_height;
        if (height_error < 0) height_error = -height_error;
        exit !(chassis_len > 0 && cabinet_len > 0 && cabinet_width > 0 &&
          gap >= 0 && lidar_y <= 0 && lidar_height > 0 &&
          x_error <= 0.002 && rear_error <= 0.002 && height_error <= 0.002);
      }'; then
    echo "Invalid chassis/cabinet/LiDAR geometry." >&2
    exit 1
  fi
  echo "Vehicle geometry: chassis=${CHASSIS_LENGTH_M}x${CHASSIS_WIDTH_M}x${CHASSIS_HEIGHT_M} m"
  echo "Cabinet geometry: length=${CABINET_LENGTH_M} width=${CABINET_WIDTH_M} front_gap=${CABINET_FRONT_GAP_M} m"
  echo "LiDAR mount: chassis_rear=${LIDAR_FROM_REAR_M} m height=${LIDAR_HEIGHT_M} m"
  echo "base_link -> nav_lidar: xyz=(${LIDAR_X_M},${LIDAR_Y_M},${LIDAR_Z_M})"
  echo "base_link -> nav_lidar: rpy=(${LIDAR_ROLL_RAD},${LIDAR_PITCH_RAD},${LIDAR_YAW_RAD}) rad"
}

for path in "${NAV2_PARAMS}" "${NAV_YAML}" "${TF_CHECKER}" "${LOCALIZATION_VERIFIER}" \
  "${NAV2_LIFECYCLE}" "${OBSTACLE_REPUBLISHER}" "${ALIGNED_GOAL_ADAPTER}" \
  "${GOAL_MARKER}" "${OBSTACLE_ALARM}" \
  "${NO_SPIN_BT}" "${NO_SPIN_THROUGH_BT}" "${NAV_FASTDDS_PROFILE}"; do
  if [[ ! -r "${path}" ]]; then
    echo "Required file is missing or unreadable: ${path}" >&2
    exit 1
  fi
done

mkdir -p "${ROS_ROOT}/logs"
validate_vehicle_geometry
if [[ "${CHECK_GEOMETRY_ONLY:-false}" == "true" ]]; then
  exit 0
fi

run_ros() {
  "${ROS_ENV}" env FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" "$@"
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
    action_info="$(run_ros ros2 action info /navigate_to_pose 2>/dev/null || true)"
    status_info="$(run_ros ros2 topic info /aligned_goal_status 2>/dev/null || true)"
    if grep -qx '/aligned_nav_goal_adapter' <<<"${nodes}" \
      && grep -Eq 'Action servers: [1-9]' <<<"${action_info}" \
      && grep -Eq 'Publisher count: [1-9]' <<<"${status_info}"; then
      echo "Goal adapter is visible as /navigate_to_pose action server."
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
  ps -eo pid=,args= | awk '
    /planner_server|controller_server|smoother_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|collision_monitor|lifecycle_manager_navigation|nav_obstacle_cloud_republisher.py|aligned_nav_goal_adapter.py|nav_goal_marker_from_plan.py|nav_obstacle_block_alarm.py|ndt_goal_controller.py|nav_motion_safety_gate.py|static_transform_publisher.*nav_lidar/ &&
    $0 !~ /awk/ &&
    $0 !~ /seeed_activate_nav2_after_initialpose/ {print $1}
  ' | xargs -r kill 2>/dev/null || true
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
  ps -eo pid=,args= | awk -v name="${node_name}" '
    index($0, name) &&
    $0 !~ /awk/ &&
    $0 !~ /seeed_activate_nav2_after_initialpose/ {print $1}
  ' | xargs -r kill 2>/dev/null || true
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
      echo "Starting /${node_name} (attempt ${attempt}/3)..."
      stop_managed_node "${node_name}"
      start_managed_node "${node_name}"
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
  local deadline=$((SECONDS + 75))
  local action output
  local actions=(/follow_path /compute_path_to_pose /compute_path_through_poses)

  while (( SECONDS < deadline )); do
    local all_ready=true
    for action in "${actions[@]}"; do
      output="$(timeout 6 "${ROS_ENV}" env \
        FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
        ros2 action info "${action}" 2>/dev/null || true)"
      if ! grep -Eq 'Action servers: [1-9]' <<<"${output}"; then
        all_ready=false
        break
      fi
    done
    if [[ "${all_ready}" == true ]]; then
      echo "Controller and planner action servers are visible before BT activation."
      return 0
    fi
    echo "Waiting for controller/planner action discovery..."
    sleep 3
  done

  for action in "${actions[@]}"; do
    echo "--- ${action}" >&2
    timeout 6 "${ROS_ENV}" env \
      FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
      ros2 action info "${action}" >&2 || true
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
      -p output_topic:=/cmd_vel \
      -p localization_timeout:=10.0 \
      -p odom_timeout:=0.30 \
      -p chassis_timeout:=0.50 \
      -p max_angular:=0.18 \
      -p minimum_linear_for_turn:=0.012 \
      -p max_motion_curvature:=2.25 \
      -p curvature_slack:=0.015
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
  ros2 topic echo --once /relocalization_odom --qos-reliability best_effort \
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
start_node nav_obstacle_cloud_republisher python3 "${OBSTACLE_REPUBLISHER}" --ros-args \
  -p output_frame:=base_link \
  -p lidar_x_m:="${LIDAR_X_M}" -p lidar_y_m:="${LIDAR_Y_M}" \
  -p lidar_z_m:="${LIDAR_Z_M}" -p lidar_roll_rad:="${LIDAR_ROLL_RAD}" \
  -p lidar_pitch_rad:="${LIDAR_PITCH_RAD}" \
  -p lidar_yaw_rad:="${LIDAR_YAW_RAD}" \
  -p self_min_x:=-0.38 -p self_max_x:=0.38 -p self_half_width:=0.27 \
  -p cabinet_min_x:=0.39 -p cabinet_max_x:=0.94 \
  -p cabinet_min_y:=-0.20 -p cabinet_max_y:=0.20 \
  -p cabinet_min_z:=-0.03 -p cabinet_max_z:=0.75 \
  -p point_stride:=12

obstacle_ready=false
for attempt in $(seq 1 12); do
  timeout 5 "${ROS_ENV}" env FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
    ros2 topic echo --once /nav_obstacle_cloud_status \
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
ps -eo pid=,args= | awk '
  /nav2_map_server.*map_server/ && $0 !~ /awk/ {print $1}
' | xargs -r kill 2>/dev/null || true
sleep 1
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

for node in bt_navigator waypoint_follower velocity_smoother collision_monitor; do
  if ! start_and_activate_managed_node "${node}"; then
    echo "Failed to start /${node} after three attempts. Motion remains DISARMED." >&2
    exit 1
  fi
done

echo "[8/11] All managed Nav2 nodes reached active [3]."

echo "[9/11] Starting the persistent Nav2 goal marker..."
goal_adapter_ready=false
for attempt in 1 2 3; do
  stop_managed_node aligned_nav_goal_adapter
  start_node aligned_nav_goal_adapter python3 "${ALIGNED_GOAL_ADAPTER}" --ros-args \
    -p approach_distance:=0.70 -p min_approach_distance:=0.45 \
    -p approach_step:=0.05 -p maximum_cost:=90 -p feedback_period:=0.20
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
start_node nav_goal_marker python3 "${GOAL_MARKER}" --ros-args \
  -p goal_tolerance:=0.08 -p text_offset:=0.70 -p pulse_period:=0.40

echo "[10/11] Starting the 60-second persistent-obstacle alarm..."
start_node nav_obstacle_block_alarm python3 "${OBSTACLE_ALARM}" --ros-args \
  -p stop_radius:=1.08 -p min_height:=0.05 -p max_height:=1.60 \
  -p min_points:=3 -p alarm_timeout_sec:=60.0 -p clear_hold_sec:=0.50

wait_for_topic_connection /nav_goal_markers 1 0
wait_for_topic_connection /nav_obstacle_alarm 1 0
wait_for_topic_connection /nav_obstacle_alarm_markers 1 0
wait_for_topic_connection /cmd_vel_nav_smoothed 1 1
wait_for_topic_connection /cmd_vel_nav_collision_safe 1 1
wait_for_topic_connection /cmd_vel 1 1

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

for topic in /nav_obstacle_cloud /local_costmap/costmap \
  /global_costmap/costmap /nav_goal_markers /nav_obstacle_alarm \
  /nav_obstacle_alarm_markers /cmd_vel_nav_collision_safe /cmd_vel; do
  echo "--- ${topic}"
  timeout 10 "${ROS_ENV}" env FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
    ros2 topic info "${topic}" || true
done

echo "--- dynamic obstacle status"
final_obstacle_status="$(timeout 8 "${ROS_ENV}" env \
  FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
  ros2 topic echo --once /nav_obstacle_cloud_status 2>/dev/null || true)"
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
  ros2 topic echo --once /nav_motion_status || true
echo "--- persistent obstacle alarm status"
timeout 8 "${ROS_ENV}" env FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
  ros2 topic echo --once /nav_obstacle_alarm_status || true

echo
echo "Nav2 dynamic avoidance is ACTIVE, but physical motion remains DISARMED."
echo "Velocity chain: Nav2 -> smoother -> collision monitor -> safety gate -> /cmd_vel"
echo "Only arm after checking the local/global costmaps in hn RViz."
