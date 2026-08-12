#!/usr/bin/env bash

set -euo pipefail

ROS_ROOT=/home/seeed/ros2
NODE=${ROS_ROOT}/scripts/ks109_dual_range_node.py
LOG_DIR=${ROS_ROOT}/logs
LOG_FILE=${LOG_DIR}/ks109_dual_range.log
PID_FILE=${LOG_DIR}/ks109_dual_range.pid
ENV=${ROS_ROOT}/use_ros_env.sh
CONVERSION_DELAY_S="${CONVERSION_DELAY_S:-0.12}"
MAXIMUM_RANGE_M="${MAXIMUM_RANGE_M:-2.0}"
SENSOR_1_FRAME="${SENSOR_1_FRAME:-rear_ultrasonic_left}"
SENSOR_2_FRAME="${SENSOR_2_FRAME:-rear_ultrasonic_right}"
NEAR_FLOOR_REJECT_ENABLED="${NEAR_FLOOR_REJECT_ENABLED:-true}"
NEAR_FLOOR_REJECT_MAX_M="${NEAR_FLOOR_REJECT_MAX_M:-0.10}"
NEAR_FLOOR_REJECT_SAMPLES="${NEAR_FLOOR_REJECT_SAMPLES:-12}"
NEAR_FLOOR_REJECT_TOLERANCE_M="${NEAR_FLOOR_REJECT_TOLERANCE_M:-0.004}"

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
      -p sensor_1_frame:="${SENSOR_1_FRAME}" \
      -p sensor_2_frame:="${SENSOR_2_FRAME}" \
      -p conversion_delay_s:="${CONVERSION_DELAY_S}" \
      -p maximum_range_m:="${MAXIMUM_RANGE_M}" \
      -p near_floor_reject_enabled:="${NEAR_FLOOR_REJECT_ENABLED}" \
      -p near_floor_reject_max_m:="${NEAR_FLOOR_REJECT_MAX_M}" \
      -p near_floor_reject_samples:="${NEAR_FLOOR_REJECT_SAMPLES}" \
      -p near_floor_reject_tolerance_m:="${NEAR_FLOOR_REJECT_TOLERANCE_M}" \
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
