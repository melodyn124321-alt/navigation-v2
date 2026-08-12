#!/usr/bin/env bash
set -euo pipefail

ROS_ROOT="${ROS_ROOT:-/home/seeed/ros2}"
ROS_ENV="${ROS_ENV:-${ROS_ROOT}/use_ros_env.sh}"
MAX_FITNESS="${MAX_FITNESS:-0.18}"
RECOVERY_MAX_FITNESS="${RECOVERY_MAX_FITNESS:-0.32}"
SONAR_STOP_DISTANCE="${SONAR_STOP_DISTANCE:-0.22}"
NAV_FASTDDS_PROFILE="${NAV_FASTDDS_PROFILE:-${ROS_ROOT}/fastrtps_profile.xml}"
COSTMAP_INSPECTOR="${COSTMAP_INSPECTOR:-${ROS_ROOT}/scripts/inspect_nav_goal_costmap.py}"
READINESS_CHECKER="${READINESS_CHECKER:-${ROS_ROOT}/scripts/nav_arm_readiness_check.py}"
DYNAMIC_INSPECTOR="${DYNAMIC_INSPECTOR:-${ROS_ROOT}/scripts/inspect_nav2_dynamic_runtime.py}"
DRY_RUN="${DRY_RUN:-false}"
ARM_READINESS_TIMEOUT="${ARM_READINESS_TIMEOUT:-35}"
REQUIRE_HN_LOCAL_COSTMAP_DISPLAY="${REQUIRE_HN_LOCAL_COSTMAP_DISPLAY:-true}"

run_ros() {
  "${ROS_ENV}" env FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" "$@"
}

if [[ ! -r "${READINESS_CHECKER}" ]]; then
  echo "Warm readiness checker is missing: ${READINESS_CHECKER}; refusing to arm." >&2
  exit 1
fi
if ! readiness_output="$(timeout "$((ARM_READINESS_TIMEOUT + 10))" "${ROS_ENV}" env \
  FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
  ROS2CLI_DISABLE_DAEMON=1 \
  python3 "${READINESS_CHECKER}" \
  --timeout "${ARM_READINESS_TIMEOUT}" \
  --max-fitness "${MAX_FITNESS}" \
  --recovery-max-fitness "${RECOVERY_MAX_FITNESS}" \
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

if [[ ! -r "${DYNAMIC_INSPECTOR}" ]]; then
  echo "Dynamic/static costmap inspector is missing: ${DYNAMIC_INSPECTOR}; refusing to arm." >&2
  exit 1
fi
if ! obstacle_check="$(timeout 30 "${ROS_ENV}" env \
  FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
  python3 "${DYNAMIC_INSPECTOR}" 2>&1)"; then
  printf '%s\n' "${obstacle_check}" >&2
  echo "Static-map obstacles or live LiDAR obstacles are absent from Nav2; refusing to arm." >&2
  exit 1
fi
printf '%s\n' "${obstacle_check}"

if [[ "${REQUIRE_HN_LOCAL_COSTMAP_DISPLAY}" == "true" ]]; then
  local_display_ready=false
  for attempt in $(seq 1 10); do
    local_display_status="$(timeout 5 "${ROS_ENV}" env \
      FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
      ROS2CLI_DISABLE_DAEMON=1 ros2 topic echo --no-daemon \
      /hn_local_costmap_status std_msgs/msg/String --once 2>/dev/null || true)"
    local_display_info="$(run_ros ros2 topic info /hn_local_costmap 2>/dev/null || true)"
    local_display_publishers="$(awk '/Publisher count:/{print $3}' <<<"${local_display_info}")"
    local_display_subscribers="$(awk '/Subscription count:/{print $3}' <<<"${local_display_info}")"
    if grep -q 'data: READY ' <<<"${local_display_status}" \
        && (( ${local_display_publishers:-0} >= 1 )) \
        && (( ${local_display_subscribers:-0} >= 1 )); then
      local_display_ready=true
      echo "HN_LOCAL_COSTMAP_DISPLAY PASS publishers=${local_display_publishers} subscribers=${local_display_subscribers} ${local_display_status//$'\n'/ }"
      break
    fi
    echo "WAITING HN local costmap display (${attempt}/10): publishers=${local_display_publishers:-0} subscribers=${local_display_subscribers:-0}"
    sleep 1
  done
  if [[ "${local_display_ready}" != "true" ]]; then
    echo "HN local costmap is not visible in RViz; refusing to arm." >&2
    printf '%s\n' "${local_display_status:-no relay status}" >&2
    printf '%s\n' "${local_display_info:-no topic info}" >&2
    exit 1
  fi
fi

# A navigation run is not considered operator-visible unless RViz is attached
# to the adapter's persistent marker output. The adapter itself now publishes
# the final green NDT-confirmed arrival marker, so this check does not depend
# on discovery of the optional redundant marker node.
goal_marker_info="$(run_ros ros2 topic info /nav_goal_markers -v 2>/dev/null || true)"
goal_marker_publishers="$(awk '/Publisher count:/{print $3}' <<<"${goal_marker_info}")"
goal_marker_subscribers="$(awk '/Subscription count:/{print $3}' <<<"${goal_marker_info}")"
if (( ${goal_marker_publishers:-0} < 1 )) \
    || (( ${goal_marker_subscribers:-0} < 1 )) \
    || ! grep -q '^Node name: aligned_nav_goal_adapter$' \
      <<<"${goal_marker_info}"; then
  echo "RViz NDT-confirmed goal marker path is incomplete; refusing to arm." >&2
  printf '%s\n' "${goal_marker_info:-no /nav_goal_markers endpoints}" >&2
  exit 1
fi
echo "RVIZ_GOAL_MARKER_DISPLAY PASS publishers=${goal_marker_publishers} subscribers=${goal_marker_subscribers} source=aligned_nav_goal_adapter"

fitness="$(sed -n 's/.*WARM_READINESS PASS fitness=\([0-9.]*\).*/\1/p' \
  <<<"${readiness_output}" | tail -n 1)"
[[ -n "${fitness}" ]] || fitness="verified"
localization_mode="$(sed -n 's/.*localization_mode=\([^ ]*\).*/\1/p' \
  <<<"${readiness_output}" | tail -n 1)"
[[ -n "${localization_mode}" ]] || localization_mode="FULL"

if [[ "${DRY_RUN}" == "true" ]]; then
  echo "READY_TO_ARM dry-run passed: NDT fitness=${fitness}, Ranger error_code=0"
  echo "Motion gate was not armed."
  exit 0
fi

echo "Arming Nav2 dynamic avoidance: NDT fitness=${fitness}, Ranger error_code=0"
if [[ "${localization_mode}" == "BOUNDED_RECOVERY" ]]; then
  echo "NDT_BOUNDED_RECOVERY: speed<=0.08m/s distance<=0.50m until " \
    "fitness<=0.18 for consecutive samples; LiDAR/ultrasonic remain active."
fi
arm_response=""
arm_attempts=1
[[ "${localization_mode}" == "BOUNDED_RECOVERY" ]] && arm_attempts=6
for arm_attempt in $(seq 1 "${arm_attempts}"); do
  arm_response="$(run_ros ros2 service call /set_nav_motion_enabled \
    std_srvs/srv/SetBool '{data: true}' 2>&1 || true)"
  printf '%s\n' "${arm_response}"
  if grep -Eq 'success=(True|true)' <<<"${arm_response}"; then
    break
  fi
  # The readiness sample and service callback are asynchronous. In bounded
  # recovery only, retry a transient fitness-window miss; every other refusal
  # remains an immediate hard failure.
  if [[ "${localization_mode}" != "BOUNDED_RECOVERY" ]] \
      || ! grep -q 'NDT fitness has not completed healthy hysteresis' \
        <<<"${arm_response}" \
      || [[ "${arm_attempt}" -eq "${arm_attempts}" ]]; then
    echo "Motion gate did not confirm arming; refusing to continue." >&2
    exit 1
  fi
  echo "WAITING bounded NDT arming window attempt=${arm_attempt}/${arm_attempts}"
  sleep 1
done
echo "Armed. The gate still publishes zero until Nav2 sends a fresh collision-checked command."
