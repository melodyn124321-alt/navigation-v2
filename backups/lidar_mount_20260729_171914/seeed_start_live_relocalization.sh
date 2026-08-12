#!/usr/bin/env bash
set -euo pipefail

ROS_ROOT="${ROS_ROOT:-/home/seeed/ros2}"
ROS_ENV="${ROS_ENV:-${ROS_ROOT}/use_ros_env.sh}"
PCD_PATH="${1:-${ROS_ROOT}/maps/replay/fastlio_map_manual_001_level.pcd}"
NAV_YAML="${2:-}"
PUBLISH_GRID="${PUBLISH_GRID:-true}"
VERIFY_LIDAR_MOUNT="${VERIFY_LIDAR_MOUNT:-${VERIFY_LIDAR_LEVEL:-true}}"
EXPECTED_LIDAR_ROLL_DEG="${EXPECTED_LIDAR_ROLL_DEG:-15.732}"
EXPECTED_LIDAR_PITCH_DEG="${EXPECTED_LIDAR_PITCH_DEG:--0.611}"
LIDAR_MOUNT_TOLERANCE_DEG="${LIDAR_MOUNT_TOLERANCE_DEG:-0.75}"
LIDAR_MOUNT_CHECK="${LIDAR_MOUNT_CHECK:-${ROS_ROOT}/scripts/verify_live_livox_mount.py}"

if [ ! -f "${PCD_PATH}" ]; then
  echo "PCD map not found: ${PCD_PATH}" >&2
  exit 1
fi
if [ ! -s "${PCD_PATH}" ]; then
  echo "PCD map is empty: ${PCD_PATH}" >&2
  exit 1
fi
if ! grep -aEq '^POINTS [1-9][0-9]*$' "${PCD_PATH}"; then
  echo "PCD header has no positive POINTS count: ${PCD_PATH}" >&2
  exit 1
fi
if ! grep -aEq '^DATA (ascii|binary)$' "${PCD_PATH}"; then
  echo "PCD encoding is unsupported by the NDT localizer: ${PCD_PATH}" >&2
  echo "Required: DATA ascii or DATA binary." >&2
  exit 1
fi
if [ -n "${NAV_YAML}" ] && [ ! -f "${NAV_YAML}" ]; then
  echo "Navigation YAML not found: ${NAV_YAML}" >&2
  exit 1
fi

mkdir -p "${ROS_ROOT}/logs"

"${ROS_ROOT}/scripts/seeed_stop_ros_stack.sh"

echo "Starting Livox driver..."
nohup "${ROS_ENV}" ros2 launch livox_ros_driver2 msg_MID360_launch.py \
  > "${ROS_ROOT}/logs/live_livox.log" 2>&1 &
sleep 5

if [ "${VERIFY_LIDAR_MOUNT}" = "true" ]; then
  echo "Checking the stationary live LiDAR against the calibrated roll/pitch..."
  if ! "${ROS_ENV}" python3 "${LIDAR_MOUNT_CHECK}" \
    --expected-roll "${EXPECTED_LIDAR_ROLL_DEG}" \
    --expected-pitch "${EXPECTED_LIDAR_PITCH_DEG}" \
    --max-angle-error "${LIDAR_MOUNT_TOLERANCE_DEG}"; then
    echo "LiDAR calibrated mount check failed. Relocalization remains stopped." >&2
    "${ROS_ROOT}/scripts/seeed_stop_ros_stack.sh" || true
    exit 4
  fi
else
  echo "WARNING: live LiDAR calibrated mount check is disabled for diagnostics." >&2
fi

echo "Starting Fast-LIO live odometry/cloud source..."
nohup "${ROS_ENV}" ros2 launch fast_lio mapping.launch.py rviz:=false \
  > "${ROS_ROOT}/logs/live_fastlio.log" 2>&1 &
sleep 8

if [ -n "${NAV_YAML}" ]; then
  echo "Starting map_server: ${NAV_YAML}"
  nohup "${ROS_ENV}" ros2 run nav2_map_server map_server --ros-args \
    -p yaml_filename:="${NAV_YAML}" \
    -p topic_name:=/map \
    -p frame_id:=map \
    > "${ROS_ROOT}/logs/live_map_server.log" 2>&1 &
  sleep 3
  "${ROS_ENV}" ros2 lifecycle set /map_server configure || true
  sleep 1
  "${ROS_ENV}" ros2 lifecycle set /map_server activate || true
fi

echo "Starting PCD NDT relocalization: ${PCD_PATH}"
NDT_LOG="${ROS_ROOT}/logs/live_pcd_ndt_localization.log"
nohup "${ROS_ENV}" ros2 launch pcd_ndt_localization localization.launch.py \
  map_path:="${PCD_PATH}" \
  publish_grid_map:="${PUBLISH_GRID}" \
  save_grid_map:=false \
  > "${NDT_LOG}" 2>&1 &
NDT_PID=$!
echo "${NDT_PID}" > "${ROS_ROOT}/logs/live_pcd_ndt_localization.pid"

sleep 5
if ! kill -0 "${NDT_PID}" 2>/dev/null || \
   ! "${ROS_ENV}" ros2 node list | grep -qx '/pcd_ndt_localizer'; then
  echo "ERROR: PCD NDT localizer failed to start." >&2
  tail -100 "${NDT_LOG}" >&2
  exit 1
fi
if grep -Eq 'failed to load PCD map|No points to read|FATAL|process has died' "${NDT_LOG}"; then
  echo "ERROR: PCD NDT localizer reported a startup failure." >&2
  tail -100 "${NDT_LOG}" >&2
  exit 1
fi
echo "Verification:"
"${ROS_ENV}" ros2 node list | sort
echo "--- /cloud_registered_body"
"${ROS_ENV}" ros2 topic info -v /cloud_registered_body || true
echo "--- /Odometry"
"${ROS_ENV}" ros2 topic info -v /Odometry || true
echo "--- /relocalization_pose"
if "${ROS_ENV}" ros2 topic list | grep -qx '/relocalization_pose'; then
  "${ROS_ENV}" ros2 topic info -v /relocalization_pose
else
  echo "/relocalization_pose is not advertised yet. This is normal before the first accepted NDT result."
fi
echo "NDT localizer is running. Use RViz 2D Pose Estimate on hn, then verify /relocalization_pose publishes."
