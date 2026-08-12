#!/usr/bin/env bash
set -euo pipefail

ROS_ROOT="${ROS_ROOT:-/home/seeed/ros2}"
ROS_ENV="${ROS_ENV:-${ROS_ROOT}/use_ros_env.sh}"
ACTION="${1:-status}"
FASTLIO_LOG="${ROS_ROOT}/logs/live_fastlio.log"
WATCHDOG_LOG="${ROS_ROOT}/logs/fastlio_watchdog.log"
TRIP_FILE="${ROS_ROOT}/logs/fastlio_watchdog.trip"
LAUNCH_PID_FILE="${ROS_ROOT}/logs/live_fastlio_launch.pid"
WATCHDOG_PID_FILE="${ROS_ROOT}/logs/fastlio_watchdog.pid"

read_pid() {
  local path="$1"
  if [[ -r "${path}" ]]; then
    tr -d '[:space:]' < "${path}"
  fi
}

pid_alive() {
  local pid="$1"
  [[ "${pid}" =~ ^[0-9]+$ ]] && kill -0 "${pid}" 2>/dev/null
}

mapping_pids() {
  pgrep -x fastlio_mapping || true
}

stop_pid() {
  local pid="$1"
  if ! pid_alive "${pid}"; then
    return
  fi
  kill -INT "${pid}" 2>/dev/null || true
  for _ in $(seq 1 30); do
    pid_alive "${pid}" || return
    sleep 0.2
  done
  kill -TERM "${pid}" 2>/dev/null || true
}

case "${ACTION}" in
  start)
    mkdir -p "${ROS_ROOT}/logs"
    [[ -x "${ROS_ENV}" ]] || {
      echo "Missing ROS environment: ${ROS_ENV}" >&2
      exit 1
    }
    [[ -x "${ROS_ROOT}/scripts/fastlio_live_watchdog.py" ]] || {
      echo "Missing watchdog: ${ROS_ROOT}/scripts/fastlio_live_watchdog.py" >&2
      exit 1
    }
    if [[ -n "$(mapping_pids)" ]]; then
      echo "FAST-LIO is already running: $(mapping_pids | tr '\n' ' ')" >&2
      exit 2
    fi
    rm -f "${TRIP_FILE}"
    : > "${FASTLIO_LOG}"
    : > "${WATCHDOG_LOG}"
    nohup "${ROS_ENV}" python3 \
      "${ROS_ROOT}/scripts/fastlio_live_watchdog.py" \
      >>"${WATCHDOG_LOG}" 2>&1 </dev/null &
    watchdog_pid=$!
    printf '%s\n' "${watchdog_pid}" > "${WATCHDOG_PID_FILE}"
    nohup "${ROS_ENV}" ros2 launch fast_lio mapping.launch.py rviz:=false \
      >>"${FASTLIO_LOG}" 2>&1 </dev/null &
    launch_pid=$!
    printf '%s\n' "${launch_pid}" > "${LAUNCH_PID_FILE}"
    for _ in $(seq 1 60); do
      if [[ -n "$(mapping_pids)" ]] && pid_alive "${watchdog_pid}"; then
        echo "RUNNING launch_pid=${launch_pid} watchdog_pid=${watchdog_pid} mapping_pids=$(mapping_pids | tr '\n' ',')"
        exit 0
      fi
      if ! pid_alive "${launch_pid}"; then
        tail -n 80 "${FASTLIO_LOG}" >&2 || true
        stop_pid "${watchdog_pid}"
        exit 1
      fi
      sleep 0.5
    done
    echo "FAST-LIO did not start within 30 seconds." >&2
    stop_pid "${launch_pid}"
    stop_pid "${watchdog_pid}"
    exit 1
    ;;
  stop)
    watchdog_pid="$(read_pid "${WATCHDOG_PID_FILE}")"
    launch_pid="$(read_pid "${LAUNCH_PID_FILE}")"
    stop_pid "${watchdog_pid}"
    stop_pid "${launch_pid}"
    for pid in $(mapping_pids); do
      stop_pid "${pid}"
    done
    rm -f "${WATCHDOG_PID_FILE}" "${LAUNCH_PID_FILE}"
    echo "STOPPED FAST-LIO and watchdog; Livox publisher and rosbag were not signalled."
    ;;
  status)
    watchdog_pid="$(read_pid "${WATCHDOG_PID_FILE}")"
    launch_pid="$(read_pid "${LAUNCH_PID_FILE}")"
    echo "mapping_pids=$(mapping_pids | tr '\n' ',' || true)"
    if pid_alive "${launch_pid}"; then
      echo "launch=RUNNING pid=${launch_pid}"
    else
      echo "launch=STOPPED"
    fi
    if pid_alive "${watchdog_pid}"; then
      echo "watchdog=RUNNING pid=${watchdog_pid}"
    else
      echo "watchdog=STOPPED"
    fi
    if [[ -s "${TRIP_FILE}" ]]; then
      echo "trip=YES"
      cat "${TRIP_FILE}"
      exit 3
    fi
    echo "trip=NO"
    ;;
  *)
    echo "Usage: $0 {start|stop|status}" >&2
    exit 64
    ;;
esac
