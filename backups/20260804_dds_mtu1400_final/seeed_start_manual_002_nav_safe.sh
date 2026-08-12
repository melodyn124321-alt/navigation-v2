#!/usr/bin/env bash
set -euo pipefail

ROS_ROOT="${ROS_ROOT:-/home/seeed/ros2}"
ROS_ENV="${ROS_ENV:-${ROS_ROOT}/use_ros_env.sh}"
DEFAULT_PCD="${ROS_ROOT}/maps/replay/fastlio_map_manual_001_level_groundsafe_20260729.pcd"
DEFAULT_NAV_YAML="${ROS_ROOT}/maps/replay/fastlio_map_manual_001_level_groundsafe_20260729_nav.yaml"
if [[ -r "${ROS_ROOT}/maps/replay/latest_raw_livox_manual_001_target.txt" ]]; then
  read -r DEFAULT_PCD < "${ROS_ROOT}/maps/replay/latest_raw_livox_manual_001_target.txt"
fi
if [[ -r "${ROS_ROOT}/maps/replay/latest_raw_livox_manual_001_nav_target.txt" ]]; then
  read -r DEFAULT_NAV_YAML < "${ROS_ROOT}/maps/replay/latest_raw_livox_manual_001_nav_target.txt"
fi
PCD_PATH="${1:-${DEFAULT_PCD}}"
NAV_YAML="${2:-${DEFAULT_NAV_YAML}}"
NAV2_PARAMS="${NAV2_PARAMS:-${ROS_ROOT}/nav2/nav2_pcd_ndt_manual_002.yaml}"
ALLOW_MOTION="${ALLOW_MOTION:-false}"
START_RANGER="${START_RANGER:-false}"
CONFIRM_NAV_MOTION="${CONFIRM_NAV_MOTION:-}"
# The chassis is connected through a gs_usb adapter.  Linux can swap can0/can1
# after a reboot or USB replug, so resolve the interface by driver instead of
# trusting a stale numeric name.  A stale explicit CAN_IFACE is corrected by
# default; set AUTO_CORRECT_CAN_IFACE=false only for an intentional override.
CAN_IFACE="${CAN_IFACE:-auto}"
EXPECTED_CAN_DRIVER="${EXPECTED_CAN_DRIVER:-gs_usb}"
AUTO_CORRECT_CAN_IFACE="${AUTO_CORRECT_CAN_IFACE:-true}"
CAN_BITRATE="${CAN_BITRATE:-500000}"
CAN_RX_WAIT_SEC="${CAN_RX_WAIT_SEC:-2.0}"
SUDO_PASSWORD="${SUDO_PASSWORD:-}"
CHASSIS_LENGTH_M="${CHASSIS_LENGTH_M:-0.720}"
CHASSIS_WIDTH_M="${CHASSIS_WIDTH_M:-0.500}"
CHASSIS_HEIGHT_M="${CHASSIS_HEIGHT_M:-0.345}"
MEASURED_LIDAR_HEIGHT_M="${MEASURED_LIDAR_HEIGHT_M:-0.495}"
# The live body cloud places the floor 0.7486 m below its optical/IMU origin.
# Use the point-cloud calibration here; the 0.495 m tape measurement is kept
# separately above because it is not the origin used by /cloud_registered_body.
LIDAR_HEIGHT_M="${LIDAR_HEIGHT_M:-0.749}"
# Measured 0.46 m forward from the rear edge of a 0.72 m chassis:
# x = -0.36 + 0.46 = +0.10 m in a chassis-centred base_link.
BASE_TO_LIDAR_X_M="${BASE_TO_LIDAR_X_M:-0.100}"
# The sensor is 0.14 m inboard from the right edge:
# y = -0.25 + 0.14 = -0.11 m.
BASE_TO_LIDAR_Y_M="${BASE_TO_LIDAR_Y_M:--0.110}"
# Two stationary 8 s IMU measurements on 2026-07-29 agreed within 0.002 deg.
BASE_TO_LIDAR_ROLL_RAD="${BASE_TO_LIDAR_ROLL_RAD:--0.801751898}"
BASE_TO_LIDAR_PITCH_RAD="${BASE_TO_LIDAR_PITCH_RAD:--0.007094763}"
# M12 is on LiDAR -X. M12 faces vehicle left (+Y), therefore LiDAR +X
# faces vehicle right (-Y).
BASE_TO_LIDAR_YAW_RAD="${BASE_TO_LIDAR_YAW_RAD:--1.570796327}"
# Exact inverse of base_link -> LiDAR using Rz(yaw) Ry(pitch) Rx(roll).
LOCALIZATION_TO_BASE_ROLL_RAD="${LOCALIZATION_TO_BASE_ROLL_RAD:-0.007094763}"
LOCALIZATION_TO_BASE_PITCH_RAD="${LOCALIZATION_TO_BASE_PITCH_RAD:--0.801751898}"
LOCALIZATION_TO_BASE_YAW_RAD="${LOCALIZATION_TO_BASE_YAW_RAD:-1.570796327}"
LOCALIZATION_TO_BASE_X_M="${LOCALIZATION_TO_BASE_X_M:--0.115311164}"
LOCALIZATION_TO_BASE_Y_M="${LOCALIZATION_TO_BASE_Y_M:-0.468093861}"
LOCALIZATION_TO_BASE_Z_M="${LOCALIZATION_TO_BASE_Z_M:--0.592192935}"
TF_BRIDGE_INPUT_TIMEOUT="${TF_BRIDGE_INPUT_TIMEOUT:-10.0}"
KS109_CONTROL="${KS109_CONTROL:-${ROS_ROOT}/scripts/seeed_ks109_dual.sh}"
MAP_VIZ_THROTTLE="${MAP_VIZ_THROTTLE:-${ROS_ROOT}/scripts/fastlio_viz_cloud_throttle.py}"

validate_chassis_lidar_geometry() {
  if ! awk -v chassis_len="${CHASSIS_LENGTH_M}" \
    -v chassis_width="${CHASSIS_WIDTH_M}" \
    -v chassis_height="${CHASSIS_HEIGHT_M}" \
    -v lidar_x="${BASE_TO_LIDAR_X_M}" \
    -v lidar_y="${BASE_TO_LIDAR_Y_M}" \
    -v lidar_height="${LIDAR_HEIGHT_M}" '
      BEGIN {
        exit !(chassis_len > 0 && chassis_width > 0 &&
          chassis_height > 0 && lidar_height > 0);
      }'; then
    echo "Invalid chassis or LiDAR geometry." >&2
    exit 1
  fi
  echo "Chassis geometry: ${CHASSIS_LENGTH_M}x${CHASSIS_WIDTH_M}x${CHASSIS_HEIGHT_M} m"
  echo "LiDAR mechanical height measurement: ${MEASURED_LIDAR_HEIGHT_M} m"
  echo "base_link -> LiDAR: xyz=(${BASE_TO_LIDAR_X_M},${BASE_TO_LIDAR_Y_M},${LIDAR_HEIGHT_M}) m"
  echo "base_link -> LiDAR: rpy=(${BASE_TO_LIDAR_ROLL_RAD},${BASE_TO_LIDAR_PITCH_RAD},${BASE_TO_LIDAR_YAW_RAD}) rad"
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

can_driver_name() {
  local iface="$1"
  local driver_path="/sys/class/net/${iface}/device/driver"
  [[ -e "${driver_path}" ]] || return 1
  basename "$(readlink -f "${driver_path}")"
}

resolve_can_iface() {
  local requested="${CAN_IFACE}"
  local requested_driver=""
  local detected_iface=""
  local detected_driver=""
  local net_path iface driver

  if [[ "${requested}" != "auto" ]]; then
    requested_driver="$(can_driver_name "${requested}" 2>/dev/null || true)"
    if [[ "${requested_driver}" == "${EXPECTED_CAN_DRIVER}" ]]; then
      echo "CAN_INTERFACE_SELECTED iface=${requested} driver=${requested_driver}"
      return 0
    fi
    if [[ "${AUTO_CORRECT_CAN_IFACE}" != "true" ]]; then
      echo "Requested CAN interface ${requested} uses driver " \
        "${requested_driver:-missing}, expected ${EXPECTED_CAN_DRIVER}." >&2
      return 1
    fi
  fi

  for net_path in /sys/class/net/can*; do
    [[ -e "${net_path}" ]] || continue
    iface="${net_path##*/}"
    driver="$(can_driver_name "${iface}" 2>/dev/null || true)"
    if [[ "${driver}" == "${EXPECTED_CAN_DRIVER}" ]]; then
      detected_iface="${iface}"
      detected_driver="${driver}"
      break
    fi
  done
  if [[ -z "${detected_iface}" ]]; then
    echo "No CAN interface using driver ${EXPECTED_CAN_DRIVER} was found." >&2
    return 1
  fi

  CAN_IFACE="${detected_iface}"
  if [[ "${requested}" == "auto" ]]; then
    echo "CAN_INTERFACE_AUTO iface=${CAN_IFACE} driver=${detected_driver}"
  else
    echo "CAN_INTERFACE_REMAP requested=${requested} " \
      "driver=${requested_driver:-missing} selected=${CAN_IFACE} " \
      "driver=${detected_driver}"
  fi
}

ensure_can_iface() {
  resolve_can_iface
  if ! ip link show "${CAN_IFACE}" >/dev/null 2>&1; then
    echo "CAN interface not found: ${CAN_IFACE}" >&2
    return 1
  fi

  if ip -details link show "${CAN_IFACE}" | grep -q "can state ERROR-ACTIVE"; then
    echo "${CAN_IFACE} is already UP and ERROR-ACTIVE."
  else
    echo "Bringing up ${CAN_IFACE} at ${CAN_BITRATE} bps..."
    run_sudo ip link set "${CAN_IFACE}" down 2>/dev/null || true
    run_sudo ip link set "${CAN_IFACE}" type can \
      bitrate "${CAN_BITRATE}" restart-ms 100
    run_sudo ip link set "${CAN_IFACE}" up
    ip -details link show "${CAN_IFACE}"
  fi

  local rx_path="/sys/class/net/${CAN_IFACE}/statistics/rx_packets"
  local rx_before rx_after deadline
  if [[ ! -r "${rx_path}" ]]; then
    echo "CAN RX counter is unavailable: ${rx_path}" >&2
    return 1
  fi
  rx_before=$(<"${rx_path}")
  deadline=$(awk -v now="$(date +%s.%N)" -v wait="${CAN_RX_WAIT_SEC}" \
    'BEGIN { printf "%.9f", now + wait }')
  while awk -v now="$(date +%s.%N)" -v end="${deadline}" \
      'BEGIN { exit !(now < end) }'; do
    sleep 0.10
    rx_after=$(<"${rx_path}")
    if (( rx_after > rx_before )); then
      echo "CAN_RX_LIVE iface=${CAN_IFACE} packets_delta=$((rx_after-rx_before))"
      return 0
    fi
  done
  rx_after=$(<"${rx_path}")
  echo "CAN_RX_DEAD iface=${CAN_IFACE} packets_delta=$((rx_after-rx_before)) " \
    "within=${CAN_RX_WAIT_SEC}s; refusing to start Ranger/Nav2 motion." >&2
  echo "Check chassis main power, release the physical E-stop, and inspect " \
    "CAN_H/CAN_L/common-ground/connectors before retrying." >&2
  return 1
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
if [[ ! -x "${KS109_CONTROL}" ]]; then
  echo "KS109 control script is missing or not executable: ${KS109_CONTROL}" >&2
  exit 1
fi
if [[ ! -x "${MAP_VIZ_THROTTLE}" ]]; then
  echo "Map visualization throttle is missing or not executable: ${MAP_VIZ_THROTTLE}" >&2
  exit 1
fi

mkdir -p "${ROS_ROOT}/logs"
validate_chassis_lidar_geometry
if [[ "${CHECK_GEOMETRY_ONLY:-false}" == "true" ]]; then
  exit 0
fi
if [[ "${CHECK_CAN_ONLY:-false}" == "true" ]]; then
  ensure_can_iface
  exit 0
fi

# Validate the physical chassis link before stopping a healthy ROS stack or
# starting LiDAR/NDT processes.  Previously a dead CAN bus was detected only
# after those processes had started, leaving a misleading half-started system
# with LOCALIZED NDT output but no odom->base_link or map->odom TF chain.
if [[ "${START_RANGER}" == "true" ]]; then
  echo "Preflighting live Ranger CAN traffic before starting the ROS stack..."
  ensure_can_iface
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

echo "Starting the two rear KS109 sensors..."
"${KS109_CONTROL}" start
if ! timeout 8 "${ROS_ENV}" ros2 topic echo \
  /ultrasonic/healthy std_msgs/msg/Bool --once \
  | grep -q "data: true"; then
  echo "The KS109 health topic is missing or unhealthy. Navigation remains stopped." >&2
  "${KS109_CONTROL}" status >&2 || true
  "${KS109_CONTROL}" stop || true
  exit 3
fi
for topic in /ultrasonic/sensor_1/range /ultrasonic/sensor_2/range; do
  if ! timeout 8 "${ROS_ENV}" ros2 topic echo \
    "${topic}" sensor_msgs/msg/Range --once \
    >/dev/null 2>&1; then
    echo "No live range message on ${topic}. Navigation remains stopped." >&2
    "${KS109_CONTROL}" stop || true
    exit 3
  fi
done
echo "Both KS109 range topics are live."

echo "Starting Livox driver..."
nohup "${ROS_ENV}" ros2 launch livox_ros_driver2 msg_MID360_launch.py \
  > "${ROS_ROOT}/logs/nav_livox.log" 2>&1 &
sleep 5

echo "LiDAR gravity roll/pitch startup check is removed."
echo "Static base_link -> LiDAR calibration remains in use for NDT and obstacle transforms."

echo "Starting Fast-LIO live odometry/cloud source..."
nohup "${ROS_ENV}" ros2 launch fast_lio mapping.launch.py rviz:=false \
  > "${ROS_ROOT}/logs/nav_fastlio.log" 2>&1 &
sleep 8

echo "Starting chassis-forward Fast-LIO odometry..."
nohup "${ROS_ENV}" python3 "${ROS_ROOT}/scripts/fastlio_chassis_odometry.py" --ros-args \
  -p base_to_lidar_x_m:="${BASE_TO_LIDAR_X_M}" \
  -p base_to_lidar_y_m:="${BASE_TO_LIDAR_Y_M}" \
  -p base_to_lidar_z_m:="${LIDAR_HEIGHT_M}" \
  -p base_to_lidar_roll_deg:="-45.937" \
  -p base_to_lidar_pitch_deg:="-0.4065" \
  -p base_to_lidar_yaw_deg:="-90.0" \
  > "${ROS_ROOT}/logs/nav_fastlio_chassis_odometry.log" 2>&1 &
sleep 1

echo "Starting PCD-NDT localization with PCD: ${PCD_PATH}"
nohup "${ROS_ENV}" ros2 launch pcd_ndt_localization localization.launch.py \
  map_path:="${PCD_PATH}" \
  publish_grid_map:=true \
  save_grid_map:=false \
  > "${ROS_ROOT}/logs/nav_pcd_ndt_localization.log" 2>&1 &
sleep 4

echo "Starting low-bandwidth RViz map cloud..."
nohup "${ROS_ENV}" python3 "${MAP_VIZ_THROTTLE}" --ros-args \
  -r __node:=map_cloud_viz_throttle \
  -p input_topic:=/map_cloud \
  -p output_topic:=/map_cloud_viz \
  -p max_rate_hz:=0.2 \
  -p point_stride:=8 \
  -p output_reliability:=reliable \
  -p output_durability:=transient_local \
  > "${ROS_ROOT}/logs/nav_map_cloud_viz.log" 2>&1 &
sleep 2
if ! "${ROS_ENV}" ros2 node list | grep -qx '/map_cloud_viz_throttle'; then
  echo "Low-bandwidth RViz map publisher failed to start." >&2
  tail -80 "${ROS_ROOT}/logs/nav_map_cloud_viz.log" >&2 || true
  exit 5
fi

echo "Starting low-bandwidth RViz aligned cloud..."
nohup "${ROS_ENV}" python3 "${MAP_VIZ_THROTTLE}" --ros-args \
  -r __node:=aligned_cloud_viz_throttle \
  -p input_topic:=/aligned_cloud \
  -p output_topic:=/aligned_cloud_viz \
  -p max_rate_hz:=2.0 \
  -p point_stride:=8 \
  -p output_reliability:=best_effort \
  -p output_durability:=volatile \
  > "${ROS_ROOT}/logs/nav_aligned_cloud_viz.log" 2>&1 &
sleep 1
if ! "${ROS_ENV}" ros2 node list | grep -qx '/aligned_cloud_viz_throttle'; then
  echo "Low-bandwidth RViz aligned-cloud publisher failed to start." >&2
  tail -80 "${ROS_ROOT}/logs/nav_aligned_cloud_viz.log" >&2 || true
  exit 5
fi

if [[ "${START_RANGER}" == "true" ]]; then
  # Ranger publishes the authoritative odom -> base_link TF itself.  This
  # avoids a second ROS participant republishing /odom only for TF.
  echo "Starting Ranger base driver with native odom TF..."
  nohup "${ROS_ENV}" ros2 launch ranger_base ranger_mini_v2.launch.py \
    port_name:="${CAN_IFACE}" \
    publish_odom_tf:=true \
    > "${ROS_ROOT}/logs/nav_ranger_base.log" 2>&1 &
  sleep 3
  if ! timeout 8 "${ROS_ENV}" ros2 topic echo \
      /odom nav_msgs/msg/Odometry --once \
      >/tmp/nav_ranger_odom_check.txt 2>&1; then
    cat /tmp/nav_ranger_odom_check.txt >&2 || true
    echo "Ranger started but did not publish /odom; navigation remains stopped." >&2
    echo "Check chassis power, physical E-stop, CAN wiring, and nav_ranger_base.log." >&2
    exit 4
  fi
  echo "Ranger /odom and native odom->base_link TF source are live."
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
if ! pgrep -f '/nav_tf_bridge/map_odom_tf_bridge|ros2 run nav_tf_bridge map_odom_tf_bridge' \
    >/dev/null; then
  echo "map_odom_tf_bridge exited during startup; navigation remains stopped." >&2
  tail -80 "${ROS_ROOT}/logs/nav_map_odom_tf_bridge.log" >&2 || true
  exit 5
fi
echo "map_odom_tf_bridge is running and will publish map->odom after LOCALIZED."

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
