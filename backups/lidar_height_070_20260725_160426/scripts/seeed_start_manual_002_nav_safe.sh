#!/usr/bin/env bash
set -euo pipefail

ROS_ROOT="${ROS_ROOT:-/home/seeed/ros2}"
ROS_ENV="${ROS_ENV:-${ROS_ROOT}/use_ros_env.sh}"
PCD_PATH="${1:-${ROS_ROOT}/maps/replay/fastlio_map_manual_001.pcd}"
NAV_YAML="${2:-${ROS_ROOT}/maps/replay/fastlio_map_manual_001_nav_current.yaml}"
NAV2_PARAMS="${NAV2_PARAMS:-${ROS_ROOT}/nav2/nav2_pcd_ndt_manual_002.yaml}"
ALLOW_MOTION="${ALLOW_MOTION:-false}"
START_RANGER="${START_RANGER:-false}"
CONFIRM_NAV_MOTION="${CONFIRM_NAV_MOTION:-}"
CAN_IFACE="${CAN_IFACE:-can0}"
CAN_BITRATE="${CAN_BITRATE:-500000}"
SUDO_PASSWORD="${SUDO_PASSWORD:-}"
CHASSIS_LENGTH_M="${CHASSIS_LENGTH_M:-0.720}"
CHASSIS_WIDTH_M="${CHASSIS_WIDTH_M:-0.500}"
CHASSIS_HEIGHT_M="${CHASSIS_HEIGHT_M:-0.345}"
CABINET_LENGTH_M="${CABINET_LENGTH_M:-0.480}"
CABINET_WIDTH_M="${CABINET_WIDTH_M:-0.360}"
CABINET_FRONT_GAP_M="${CABINET_FRONT_GAP_M:-0.050}"
LIDAR_FROM_REAR_M="${LIDAR_FROM_REAR_M:-1.250}"
LIDAR_HEIGHT_M="${LIDAR_HEIGHT_M:-0.730}"
BASE_TO_LIDAR_X_M="${BASE_TO_LIDAR_X_M:-0.890}"
BASE_TO_LIDAR_Y_M="${BASE_TO_LIDAR_Y_M:--0.050}"
BASE_TO_LIDAR_ROLL_RAD="${BASE_TO_LIDAR_ROLL_RAD:--0.00005236}"
BASE_TO_LIDAR_PITCH_RAD="${BASE_TO_LIDAR_PITCH_RAD:-0.03940953}"
BASE_TO_LIDAR_YAW_RAD="${BASE_TO_LIDAR_YAW_RAD:-3.141592653589793}"
LOCALIZATION_TO_BASE_ROLL_RAD="${LOCALIZATION_TO_BASE_ROLL_RAD:--0.00005240}"
LOCALIZATION_TO_BASE_PITCH_RAD="${LOCALIZATION_TO_BASE_PITCH_RAD:-0.03940953}"
LOCALIZATION_TO_BASE_YAW_RAD="${LOCALIZATION_TO_BASE_YAW_RAD:-3.14159059}"
LOCALIZATION_TO_BASE_X_M="${LOCALIZATION_TO_BASE_X_M:-0.9180705}"
LOCALIZATION_TO_BASE_Y_M="${LOCALIZATION_TO_BASE_Y_M:--0.0499636}"
LOCALIZATION_TO_BASE_Z_M="${LOCALIZATION_TO_BASE_Z_M:--0.6943704}"
TF_BRIDGE_INPUT_TIMEOUT="${TF_BRIDGE_INPUT_TIMEOUT:-10.0}"
VERIFY_LIDAR_MOUNT="${VERIFY_LIDAR_MOUNT:-${VERIFY_LIDAR_LEVEL:-true}}"
EXPECTED_LIDAR_ROLL_DEG="${EXPECTED_LIDAR_ROLL_DEG:--0.003}"
EXPECTED_LIDAR_PITCH_DEG="${EXPECTED_LIDAR_PITCH_DEG:-2.258}"
LIDAR_MOUNT_TOLERANCE_DEG="${LIDAR_MOUNT_TOLERANCE_DEG:-0.75}"
LIDAR_MOUNT_CHECK="${LIDAR_MOUNT_CHECK:-${ROS_ROOT}/scripts/verify_live_livox_mount.py}"

validate_vehicle_geometry() {
  if ! awk -v chassis_len="${CHASSIS_LENGTH_M}" \
    -v cabinet_len="${CABINET_LENGTH_M}" \
    -v cabinet_width="${CABINET_WIDTH_M}" \
    -v gap="${CABINET_FRONT_GAP_M}" \
    -v rear="${LIDAR_FROM_REAR_M}" \
    -v lidar_x="${BASE_TO_LIDAR_X_M}" \
    -v lidar_y="${BASE_TO_LIDAR_Y_M}" \
    -v lidar_height="${LIDAR_HEIGHT_M}" '
      BEGIN {
        expected_x = chassis_len / 2.0 + gap + cabinet_len;
        x_error = lidar_x - expected_x; if (x_error < 0) x_error = -x_error;
        rear_error = rear - (chassis_len + gap + cabinet_len);
        if (rear_error < 0) rear_error = -rear_error;
        exit !(chassis_len > 0 && cabinet_len > 0 && cabinet_width > 0 &&
          gap >= 0 && lidar_height > 0 && lidar_y <= 0 &&
          x_error <= 0.002 && rear_error <= 0.002);
      }'; then
    echo "Invalid chassis/cabinet/LiDAR geometry." >&2
    exit 1
  fi
  echo "Vehicle geometry: chassis=${CHASSIS_LENGTH_M}x${CHASSIS_WIDTH_M}x${CHASSIS_HEIGHT_M} m"
  echo "Cabinet geometry: length=${CABINET_LENGTH_M} width=${CABINET_WIDTH_M} front_gap=${CABINET_FRONT_GAP_M} m"
  echo "LiDAR mount: chassis_rear=${LIDAR_FROM_REAR_M} m height=${LIDAR_HEIGHT_M} m"
  echo "base_link -> LiDAR: xyz=(${BASE_TO_LIDAR_X_M},${BASE_TO_LIDAR_Y_M},${LIDAR_HEIGHT_M}) m"
  echo "base_link -> LiDAR: rpy=(${BASE_TO_LIDAR_ROLL_RAD},${BASE_TO_LIDAR_PITCH_RAD},${BASE_TO_LIDAR_YAW_RAD}) rad"
  echo "Expected live gravity tilt: roll=${EXPECTED_LIDAR_ROLL_DEG} deg pitch=${EXPECTED_LIDAR_PITCH_DEG} deg"
}

run_sudo() {
  if sudo -n true 2>/dev/null; then
    sudo "$@"
  elif [[ -n "${SUDO_PASSWORD}" ]]; then
    printf '%s\n' "${SUDO_PASSWORD}" | sudo -S "$@"
  else
    echo "sudo password required. Re-run with SUDO_PASSWORD=... or bring up ${CAN_IFACE} manually." >&2
    return 1
  fi
}

ensure_can_iface() {
  if ! ip link show "${CAN_IFACE}" >/dev/null 2>&1; then
    echo "CAN interface not found: ${CAN_IFACE}" >&2
    return 1
  fi

  if ip -details link show "${CAN_IFACE}" | grep -q "can state ERROR-ACTIVE"; then
    echo "${CAN_IFACE} is already UP and ERROR-ACTIVE."
    return 0
  fi

  echo "Bringing up ${CAN_IFACE} at ${CAN_BITRATE} bps..."
  run_sudo ip link set "${CAN_IFACE}" down 2>/dev/null || true
  run_sudo ip link set "${CAN_IFACE}" type can bitrate "${CAN_BITRATE}"
  run_sudo ip link set "${CAN_IFACE}" up
  ip -details link show "${CAN_IFACE}"
}

if [[ ! -f "${PCD_PATH}" ]]; then
  echo "PCD not found: ${PCD_PATH}" >&2
  exit 1
fi
if [[ ! -f "${NAV2_PARAMS}" ]]; then
  echo "Nav2 params not found: ${NAV2_PARAMS}" >&2
  exit 1
fi
if [[ ! -x "${ROS_ENV}" ]]; then
  echo "ROS env script not executable: ${ROS_ENV}" >&2
  exit 1
fi

mkdir -p "${ROS_ROOT}/logs"
validate_vehicle_geometry
if [[ "${CHECK_GEOMETRY_ONLY:-false}" == "true" ]]; then
  exit 0
fi

if [[ ! -s "${NAV_YAML}" || ! -s "${NAV_YAML}.pgm" ]]; then
  "${ROS_ROOT}/scripts/seeed_prepare_manual_002_nav_map.sh" "${PCD_PATH}" "${NAV_YAML}"
fi

CMD_VEL_TOPIC="/cmd_vel"
if [[ "${ALLOW_MOTION}" == "true" ]]; then
  if [[ "${CONFIRM_NAV_MOTION}" != "I_UNDERSTAND" ]]; then
    echo "Refusing to connect Nav2 to /cmd_vel without CONFIRM_NAV_MOTION=I_UNDERSTAND" >&2
    exit 1
  fi
  CMD_VEL_TOPIC="/cmd_vel"
fi

"${ROS_ROOT}/scripts/seeed_stop_ros_stack.sh"

echo "Starting Livox driver..."
nohup "${ROS_ENV}" ros2 launch livox_ros_driver2 msg_MID360_launch.py \
  > "${ROS_ROOT}/logs/nav_livox.log" 2>&1 &
sleep 5

if [[ "${VERIFY_LIDAR_MOUNT}" == "true" ]]; then
echo "Checking the stationary live LiDAR against the calibrated +2.258 deg pitch..."
  if ! "${ROS_ENV}" python3 "${LIDAR_MOUNT_CHECK}" \
    --expected-roll "${EXPECTED_LIDAR_ROLL_DEG}" \
    --expected-pitch "${EXPECTED_LIDAR_PITCH_DEG}" \
    --max-angle-error "${LIDAR_MOUNT_TOLERANCE_DEG}"; then
    echo "LiDAR calibrated mount check failed. Navigation remains stopped." >&2
    "${ROS_ROOT}/scripts/seeed_stop_ros_stack.sh" || true
    exit 4
  fi
else
  echo "WARNING: live LiDAR calibrated mount check is disabled." >&2
fi

echo "Starting Fast-LIO live odometry/cloud source..."
nohup "${ROS_ENV}" ros2 launch fast_lio mapping.launch.py rviz:=false \
  > "${ROS_ROOT}/logs/nav_fastlio.log" 2>&1 &
sleep 8

echo "Starting PCD-NDT localization with PCD: ${PCD_PATH}"
nohup "${ROS_ENV}" ros2 launch pcd_ndt_localization localization.launch.py \
  map_path:="${PCD_PATH}" \
  publish_grid_map:=false \
  save_grid_map:=false \
  > "${ROS_ROOT}/logs/nav_pcd_ndt_localization.log" 2>&1 &
sleep 4

if [[ "${START_RANGER}" == "true" ]]; then
  ensure_can_iface
  # Ranger publishes the authoritative odom -> base_link TF itself.  This
  # avoids a second ROS participant republishing /odom only for TF.
  echo "Starting Ranger base driver with native odom TF..."
  nohup "${ROS_ENV}" ros2 launch ranger_base ranger_mini_v2.launch.py \
    publish_odom_tf:=true \
    > "${ROS_ROOT}/logs/nav_ranger_base.log" 2>&1 &
  sleep 3
else
  echo "Ranger base driver is not started. START_RANGER=true enables it."
fi

echo "Starting low-overhead map->odom TF bridge..."
nohup "${ROS_ENV}" ros2 run nav_tf_bridge map_odom_tf_bridge --ros-args \
  -p localization_to_base_roll_rad:="${LOCALIZATION_TO_BASE_ROLL_RAD}" \
  -p localization_to_base_pitch_rad:="${LOCALIZATION_TO_BASE_PITCH_RAD}" \
  -p localization_to_base_yaw_rad:="${LOCALIZATION_TO_BASE_YAW_RAD}" \
  -p localization_to_base_x_m:="${LOCALIZATION_TO_BASE_X_M}" \
  -p localization_to_base_y_m:="${LOCALIZATION_TO_BASE_Y_M}" \
  -p localization_to_base_z_m:="${LOCALIZATION_TO_BASE_Z_M}" \
  -p input_timeout:="${TF_BRIDGE_INPUT_TIMEOUT}" \
  -p publish_rate:=10.0 \
  > "${ROS_ROOT}/logs/nav_map_odom_tf_bridge.log" 2>&1 &
sleep 2

echo "Nav2 is intentionally not started before initial localization."
echo "Set RViz 2D Pose Estimate first. After /relocalization_pose is stable, run:"
echo "  ${ROS_ROOT}/scripts/seeed_activate_nav2_after_initialpose.sh"
sleep 5

echo "Verification:"
"${ROS_ENV}" ros2 node list | sort
echo "--- important topics"
"${ROS_ENV}" ros2 topic list -t | egrep "map|costmap|plan|cmd_vel|goal|relocalization|cloud_registered|tf" || true
echo "--- safety"
echo "Nav2 is not started yet, so no navigation velocity is being generated."
if [[ "${START_RANGER}" == "true" ]]; then
  echo "Ranger base is started, but /cmd_vel is still not bridged to Nav2."
else
  echo "Ranger base driver is not started. START_RANGER=true enables it."
fi
echo "Use RViz 2D Pose Estimate first, activate Nav2, then send a Nav2 Goal only after /relocalization_pose is stable."
