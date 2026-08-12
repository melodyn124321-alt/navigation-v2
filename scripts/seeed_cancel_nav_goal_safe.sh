#!/usr/bin/env bash
set -euo pipefail

ROS_ROOT="${ROS_ROOT:-/home/seeed/ros2}"
ROS_ENV="${ROS_ENV:-${ROS_ROOT}/use_ros_env.sh}"
NAV_FASTDDS_PROFILE="${NAV_FASTDDS_PROFILE:-${ROS_ROOT}/fastrtps_profile.xml}"
DISARM_SCRIPT="${DISARM_SCRIPT:-${ROS_ROOT}/scripts/seeed_disarm_nav_motion.sh}"
ACTION_CANCELER="${ACTION_CANCELER:-${ROS_ROOT}/scripts/cancel_nav_actions_once.py}"

"${DISARM_SCRIPT}"

if [[ ! -x "${ACTION_CANCELER}" ]]; then
  echo "Action canceler is missing: ${ACTION_CANCELER}" >&2
  echo "Physical motion is DISARMED, but goal cancellation is not confirmed." >&2
  exit 3
fi
timeout 10 "${ROS_ENV}" env \
  FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
  ROS2CLI_DISABLE_DAEMON=1 \
  python3 "${ACTION_CANCELER}"
echo "Navigation goal cancellation confirmed; physical motion remains DISARMED."
