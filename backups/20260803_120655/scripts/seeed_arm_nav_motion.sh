#!/usr/bin/env bash
set -euo pipefail

ROS_ROOT="${ROS_ROOT:-/home/seeed/ros2}"
ROS_ENV="${ROS_ENV:-${ROS_ROOT}/use_ros_env.sh}"
MAX_FITNESS="${MAX_FITNESS:-0.10}"
SONAR_STOP_DISTANCE="${SONAR_STOP_DISTANCE:-0.60}"
NAV_FASTDDS_PROFILE="${NAV_FASTDDS_PROFILE:-${ROS_ROOT}/fastrtps_profile.xml}"
COSTMAP_INSPECTOR="${COSTMAP_INSPECTOR:-${ROS_ROOT}/scripts/inspect_nav_goal_costmap.py}"
READINESS_CHECKER="${READINESS_CHECKER:-${ROS_ROOT}/scripts/nav_arm_readiness_check.py}"
DRY_RUN="${DRY_RUN:-false}"

run_ros() {
  "${ROS_ENV}" env FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" "$@"
}

if [[ ! -r "${READINESS_CHECKER}" ]]; then
  echo "Warm readiness checker is missing: ${READINESS_CHECKER}; refusing to arm." >&2
  exit 1
fi
if ! readiness_output="$(timeout 22 "${ROS_ENV}" env \
  FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
  ROS2CLI_DISABLE_DAEMON=1 \
  python3 "${READINESS_CHECKER}" \
  --timeout 15 \
  --max-fitness "${MAX_FITNESS}" \
  --sonar-stop-distance "${SONAR_STOP_DISTANCE}" 2>&1)"; then
  printf '%s\n' "${readiness_output:-no readiness output}" >&2
  echo "Warm navigation readiness failed; refusing to arm." >&2
  exit 1
fi
printf '%s\n' "${readiness_output}"

last_costmap_warning_sec="$(sed -n \
  's/.*\[\([0-9][0-9]*\)\.[0-9]*\].*observation buffer has not been updated.*/\1/p' \
  "${ROS_ROOT}/logs/nav2_controller_server.log" | tail -n 1)"
now_sec="$(date +%s)"
if [[ -n "${last_costmap_warning_sec}" ]] \
  && (( now_sec - last_costmap_warning_sec <= 3 )); then
  echo "Local costmap observation buffer is stale; refusing to arm." >&2
  exit 1
fi

if [[ ! -r "${COSTMAP_INSPECTOR}" ]]; then
  echo "Costmap inspector is missing: ${COSTMAP_INSPECTOR}; refusing to arm." >&2
  exit 1
fi
if ! start_check="$(timeout 25 "${ROS_ENV}" env \
  FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
  python3 "${COSTMAP_INSPECTOR}" --require-start-passable --timeout 18 2>&1)"; then
  printf '%s\n' "${start_check}" >&2
  echo "The current Nav2 start cell is blocked; refusing to arm." >&2
  echo "Correct the 2D Pose Estimate or place the stopped robot in free space." >&2
  exit 1
fi
printf '%s\n' "${start_check}"

fitness="$(sed -n 's/.*WARM_READINESS PASS fitness=\([0-9.]*\).*/\1/p' \
  <<<"${readiness_output}" | tail -n 1)"
[[ -n "${fitness}" ]] || fitness="verified"

if [[ "${DRY_RUN}" == "true" ]]; then
  echo "READY_TO_ARM dry-run passed: NDT fitness=${fitness}, Ranger error_code=0"
  echo "Motion gate was not armed."
  exit 0
fi

echo "Arming Nav2 dynamic avoidance: NDT fitness=${fitness}, Ranger error_code=0"
arm_response="$(run_ros ros2 service call /set_nav_motion_enabled \
  std_srvs/srv/SetBool '{data: true}')"
printf '%s\n' "${arm_response}"
if ! grep -Eq 'success=(True|true)' <<<"${arm_response}"; then
  echo "Motion gate did not confirm arming; refusing to continue." >&2
  exit 1
fi
echo "Armed. The gate still publishes zero until Nav2 sends a fresh collision-checked command."
