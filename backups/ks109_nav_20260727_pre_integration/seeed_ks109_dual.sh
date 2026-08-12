#!/usr/bin/env bash

set -euo pipefail

ROS_ROOT=/home/seeed/ros2
NODE=${ROS_ROOT}/scripts/ks109_dual_range_node.py
LOG_DIR=${ROS_ROOT}/logs
LOG_FILE=${LOG_DIR}/ks109_dual_range.log
PID_FILE=${LOG_DIR}/ks109_dual_range.pid
ENV=${ROS_ROOT}/use_ros_env.sh

mkdir -p "${LOG_DIR}"

is_running() {
  [[ -f "${PID_FILE}" ]] || return 1
  local pid
  local process_state
  pid=$(<"${PID_FILE}")
  [[ "${pid}" =~ ^[0-9]+$ ]] || return 1
  kill -0 "${pid}" 2>/dev/null || return 1
  process_state=$(ps -o stat= -p "${pid}" 2>/dev/null | xargs)
  [[ -n "${process_state}" && "${process_state}" != Z* ]]
}

case "${1:-}" in
  start)
    if is_running; then
      echo "ALREADY_RUNNING pid=$(<"${PID_FILE}")"
      exit 0
    fi
    if [[ ! -r /dev/i2c-7 || ! -w /dev/i2c-7 ]]; then
      echo "ERROR: current user cannot access /dev/i2c-7" >&2
      exit 1
    fi
    nohup "${ENV}" python3 "${NODE}" \
      --ros-args \
      -p bus:=7 \
      -p sensor_1_address:=116 \
      -p sensor_2_address:=117 \
      >"${LOG_FILE}" 2>&1 </dev/null &
    echo "$!" >"${PID_FILE}"
    sleep 5
    if ! is_running; then
      : >"${PID_FILE}"
      echo "START_FAILED"
      tail -40 "${LOG_FILE}" || true
      exit 1
    fi
    echo "STARTED pid=$(<"${PID_FILE}") log=${LOG_FILE}"
    ;;
  stop)
    if ! is_running; then
      echo "NOT_RUNNING"
      exit 0
    fi
    pid=$(<"${PID_FILE}")
    kill -INT "${pid}"
    for _ in $(seq 1 30); do
      kill -0 "${pid}" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "${pid}" 2>/dev/null; then
      kill -TERM "${pid}"
    fi
    : >"${PID_FILE}"
    echo "STOPPED"
    ;;
  status)
    if is_running; then
      echo "RUNNING pid=$(<"${PID_FILE}")"
    else
      echo "NOT_RUNNING"
    fi
    tail -20 "${LOG_FILE}" 2>/dev/null || true
    ;;
  *)
    echo "Usage: $0 {start|stop|status}" >&2
    exit 2
    ;;
esac
