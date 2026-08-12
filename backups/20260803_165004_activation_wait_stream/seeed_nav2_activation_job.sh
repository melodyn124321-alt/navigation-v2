#!/usr/bin/env bash
set -euo pipefail

ROS_ROOT="${ROS_ROOT:-/home/seeed/ros2}"
ACTIVATE_SCRIPT="${ACTIVATE_SCRIPT:-${ROS_ROOT}/scripts/seeed_activate_nav2_after_initialpose.sh}"
DISARM_SCRIPT="${DISARM_SCRIPT:-${ROS_ROOT}/scripts/seeed_disarm_nav_motion.sh}"
ARM_SCRIPT="${ARM_SCRIPT:-${ROS_ROOT}/scripts/seeed_arm_nav_motion.sh}"
GOAL_ADAPTER_REFRESH="${GOAL_ADAPTER_REFRESH:-${ROS_ROOT}/scripts/seeed_refresh_nav_goal_adapter.sh}"
ROS_ENV="${ROS_ENV:-${ROS_ROOT}/use_ros_env.sh}"
NAV_FASTDDS_PROFILE="${NAV_FASTDDS_PROFILE:-${ROS_ROOT}/fastrtps_profile.xml}"
DEFAULT_NAV_YAML="${ROS_ROOT}/maps/replay/fastlio_map_manual_001_level_groundsafe_20260729_nav.yaml"
if [[ -r "${ROS_ROOT}/maps/replay/latest_raw_livox_manual_001_nav_target.txt" ]]; then
  read -r DEFAULT_NAV_YAML < "${ROS_ROOT}/maps/replay/latest_raw_livox_manual_001_nav_target.txt"
fi
LOG_DIR="${ROS_ROOT}/logs"
LOG_FILE="${LOG_DIR}/nav2_activation_full.log"
EXIT_FILE="${LOG_DIR}/nav2_activation.exit"
PID_FILE="${LOG_DIR}/nav2_activation.pid"
STATUS_FILE="${LOG_DIR}/nav2_activation.status"

atomic_write() {
  local target="$1"
  local value="$2"
  local temporary="${target}.tmp.$$"
  printf '%s\n' "${value}" > "${temporary}"
  mv -f "${temporary}" "${target}"
}

ensure_cli_daemon() {
  if timeout 10 "${ROS_ENV}" env \
    FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
    ros2 node list >/dev/null 2>&1; then
    printf 'ROS CLI daemon is healthy.\n'
    return 0
  fi

  printf 'Repairing an unresponsive ROS CLI daemon before activation...\n'
  "${ROS_ENV}" env FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
    ros2 daemon stop >/dev/null 2>&1 || true
  sleep 2
  "${ROS_ENV}" env FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
    ros2 daemon start >/dev/null
  sleep 3
  if ! timeout 12 "${ROS_ENV}" env \
    FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
    ros2 node list >/dev/null 2>&1; then
    printf 'ROS CLI daemon is still unavailable after repair.\n' >&2
    return 1
  fi
  printf 'ROS CLI daemon repaired.\n'
}

worker() {
  local nav_yaml="$1"
  local rc=125

  finish_worker() {
    rc="${1:-125}"
    trap - EXIT
    atomic_write "${EXIT_FILE}" "${rc}"
    if (( rc == 0 )); then
      atomic_write "${STATUS_FILE}" \
        "SUCCEEDED exit_code=0 finished=$(date --iso-8601=seconds)"
    else
      atomic_write "${STATUS_FILE}" \
        "FAILED exit_code=${rc} finished=$(date --iso-8601=seconds)"
    fi
    rm -f "${PID_FILE}"
    exit "${rc}"
  }
  atomic_write "${STATUS_FILE}" \
    "RUNNING pid=$$ started=$(date --iso-8601=seconds) map=${nav_yaml}"
  set +e
  "${ACTIVATE_SCRIPT}" "${nav_yaml}"
  rc=$?
  set -e
  finish_worker "${rc}"
}

show_status() {
  local pid=""
  local rc=""

  if [[ -r "${EXIT_FILE}" ]]; then
    rc="$(tr -d '[:space:]' < "${EXIT_FILE}")"
    cat "${STATUS_FILE}" 2>/dev/null || printf 'COMPLETED exit_code=%s\n' "${rc}"
    printf 'exit_file=%s\n' "${EXIT_FILE}"
    printf 'log_file=%s\n' "${LOG_FILE}"
    if [[ "${rc}" != "0" ]]; then
      printf 'FAILED is a terminal result; wait does not retry it.\n' >&2
      printf 'Inspect the log, correct the prerequisite, then run: %s start\n' \
        "${BASH_SOURCE[0]}" >&2
    fi
    [[ "${rc}" == "0" ]]
    return
  fi

  if [[ -r "${PID_FILE}" ]]; then
    pid="$(tr -d '[:space:]' < "${PID_FILE}")"
  fi
  if [[ "${pid}" =~ ^[0-9]+$ ]] && kill -0 "${pid}" 2>/dev/null; then
    cat "${STATUS_FILE}" 2>/dev/null \
      || printf 'RUNNING pid=%s exit code is not available yet\n' "${pid}"
    printf 'exit_code=PENDING\n'
    printf 'log_file=%s\n' "${LOG_FILE}"
    return 0
  fi

  cat "${STATUS_FILE}" 2>/dev/null || true
  printf 'NOT_RUNNING: no completed exit code and no live activation process\n' >&2
  printf 'Inspect: %s\n' "${LOG_FILE}" >&2
  return 2
}

start_job() {
  local nav_yaml="${1:-${DEFAULT_NAV_YAML}}"
  local old_pid=""
  local pid=""

  mkdir -p "${LOG_DIR}"
  for path in "${ACTIVATE_SCRIPT}" "${nav_yaml}"; do
    if [[ ! -r "${path}" ]]; then
      printf 'Required file is missing or unreadable: %s\n' "${path}" >&2
      exit 1
    fi
  done

  if [[ -r "${PID_FILE}" ]]; then
    old_pid="$(tr -d '[:space:]' < "${PID_FILE}")"
  fi
  if [[ "${old_pid}" =~ ^[0-9]+$ ]] && kill -0 "${old_pid}" 2>/dev/null; then
    printf 'Activation is already running with pid=%s\n' "${old_pid}" >&2
    show_status
    return 3
  fi

  if [[ -x "${DISARM_SCRIPT}" ]]; then
    printf 'Locking navigation motion before activation...\n'
    "${DISARM_SCRIPT}"
  else
    printf 'Required executable is missing: %s\n' "${DISARM_SCRIPT}" >&2
    exit 1
  fi

  ensure_cli_daemon

  # The absence of EXIT_FILE now means PENDING, never an unknown launch state.
  rm -f "${EXIT_FILE}" "${PID_FILE}"
  atomic_write "${STATUS_FILE}" \
    "STARTING requested=$(date --iso-8601=seconds) map=${nav_yaml}"
  : > "${LOG_FILE}"

  nohup "${BASH_SOURCE[0]}" --worker "${nav_yaml}" \
    >>"${LOG_FILE}" 2>&1 </dev/null &
  pid=$!
  atomic_write "${PID_FILE}" "${pid}"
  printf 'NAV2_ACTIVATION_STARTED pid=%s\n' "${pid}"
  printf 'Use: %s status\n' "${BASH_SOURCE[0]}"
  printf 'Use: %s wait 360\n' "${BASH_SOURCE[0]}"
}

wait_job() {
  local timeout_sec="${1:-360}"
  local wait_started=${SECONDS}
  local next_report=${SECONDS}
  local deadline=$((SECONDS + timeout_sec))
  local pid=""
  local stage=""

  while (( SECONDS < deadline )); do
    if [[ -r "${EXIT_FILE}" ]]; then
      show_status
      return $?
    fi
    if [[ -r "${PID_FILE}" ]]; then
      pid="$(tr -d '[:space:]' < "${PID_FILE}")"
      if [[ ! "${pid}" =~ ^[0-9]+$ ]] || ! kill -0 "${pid}" 2>/dev/null; then
        show_status
        return $?
      fi
    fi
    if (( SECONDS >= next_report )); then
      stage="$(grep -E '^\[[0-9]+/11\]' "${LOG_FILE}" 2>/dev/null \
        | tail -n 1 || true)"
      printf 'WAITING elapsed=%ss stage=%s\n' \
        "$((SECONDS - wait_started))" "${stage:-starting}"
      next_report=$((SECONDS + 10))
    fi
    sleep 2
  done
  printf 'TIMEOUT after %ss; activation may still be running\n' "${timeout_sec}" >&2
  show_status || true
  return 124
}

verify_current() {
  mkdir -p "${LOG_DIR}"
  if [[ ! -x "${DISARM_SCRIPT}" \
    || ! -x "${ARM_SCRIPT}" \
    || ! -x "${GOAL_ADAPTER_REFRESH}" ]]; then
    printf 'Required disarm, readiness, or goal-adapter refresh script is unavailable.\n' >&2
    return 1
  fi
  "${DISARM_SCRIPT}"
  if ! DRY_RUN=true "${ARM_SCRIPT}"; then
    printf 'Current Nav2 runtime failed warm verification.\n' >&2
    return 1
  fi
  if ! "${GOAL_ADAPTER_REFRESH}"; then
    printf 'RViz-facing /navigate_to_pose adapter refresh failed.\n' >&2
    return 1
  fi
  atomic_write "${EXIT_FILE}" "0"
  atomic_write "${STATUS_FILE}" \
    "SUCCEEDED exit_code=0 verified_current=$(date --iso-8601=seconds)"
  rm -f "${PID_FILE}"
  show_status
}

case "${1:-}" in
  start)
    shift
    start_job "${1:-${DEFAULT_NAV_YAML}}"
    ;;
  status)
    show_status
    ;;
  wait)
    wait_job "${2:-360}"
    ;;
  log)
    tail -n "${2:-120}" "${LOG_FILE}"
    ;;
  verify)
    verify_current
    ;;
  --worker)
    worker "${2:?navigation YAML is required}"
    ;;
  *)
    printf 'Usage: %s {start [nav.yaml]|status|wait [seconds]|log [lines]|verify}\n' "$0" >&2
    exit 2
    ;;
esac
