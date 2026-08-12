#!/usr/bin/env bash
set -euo pipefail

ROS_ROOT="${ROS_ROOT:-/home/seeed/ros2}"
ROS_ENV="${ROS_ENV:-${ROS_ROOT}/use_ros_env.sh}"
NAV_FASTDDS_PROFILE="${NAV_FASTDDS_PROFILE:-${ROS_ROOT}/fastrtps_profile.xml}"
SERVICE_DISCOVERY_TIMEOUT="${SERVICE_DISCOVERY_TIMEOUT:-5}"
SERVICE_CALL_TIMEOUT="${SERVICE_CALL_TIMEOUT:-8}"
ZERO_PUBLISH_TIMEOUT="${ZERO_PUBLISH_TIMEOUT:-5}"

run_ros_timeout() {
  local duration="$1"
  shift
  timeout --signal=INT --kill-after=2 "${duration}" \
    "${ROS_ENV}" env \
    FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
    ROS2CLI_DISABLE_DAEMON=1 \
    "$@"
}

service_type="$(
  run_ros_timeout "${SERVICE_DISCOVERY_TIMEOUT}" \
    ros2 service type /set_nav_motion_enabled 2>/dev/null || true
)"
if [[ "${service_type}" == "std_srvs/srv/SetBool" ]]; then
  if ! run_ros_timeout "${SERVICE_CALL_TIMEOUT}" \
    ros2 service call /set_nav_motion_enabled \
    std_srvs/srv/SetBool '{data: false}'; then
    echo "WARNING: the motion gate was discovered, but its disarm call timed out." >&2
  fi
else
  echo "Motion gate service is absent; it is already stopped or not yet started."
fi

# ros2 topic pub --once otherwise waits forever for its default one matching
# subscription. Bound that wait so an already-stopped Ranger/Nav2 stack cannot
# block the following full-stack shutdown.
if run_ros_timeout "${ZERO_PUBLISH_TIMEOUT}" \
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'; then
  echo "Zero velocity was delivered to a discovered /cmd_vel subscriber."
else
  echo "No /cmd_vel subscriber was discovered within ${ZERO_PUBLISH_TIMEOUT}s; continuing because the command path is absent or already stopping."
fi

echo "Navigation motion disarm sequence completed without an unbounded DDS wait."
