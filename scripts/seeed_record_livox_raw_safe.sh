#!/usr/bin/env bash
set -euo pipefail

ROS_ROOT="${ROS_ROOT:-/home/seeed/ros2}"
ROS_ENV="${ROS_ENV:-${ROS_ROOT}/use_ros_env.sh}"
QOS_FILE="${QOS_FILE:-${ROS_ROOT}/config/livox_bag_qos.yaml}"
CACHE_BYTES="${CACHE_BYTES:-536870912}"
CONTINUITY_ANALYZER="${CONTINUITY_ANALYZER:-${ROS_ROOT}/scripts/analyze_livox_bag_continuity.py}"
EXPECTED_LIDAR_ROLL_DEG="${EXPECTED_LIDAR_ROLL_DEG:--45.937}"
EXPECTED_LIDAR_PITCH_DEG="${EXPECTED_LIDAR_PITCH_DEG:--0.4065}"
LIDAR_MOUNT_TOLERANCE_DEG="${LIDAR_MOUNT_TOLERANCE_DEG:-0.75}"
ACTION="${1:-status}"
BAG_NAME="${2:-}"
OVERWRITE_EXISTING_BAG="${OVERWRITE_EXISTING_BAG:-false}"

if [[ "${ACTION}" == "start-overwrite" ]]; then
  ACTION="start"
  OVERWRITE_EXISTING_BAG="true"
fi

validate_name() {
  if [[ ! "${BAG_NAME}" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]]; then
    echo "A safe bag name is required, for example raw_livox_manual_003." >&2
    exit 1
  fi
}

paths() {
  BAG_PATH="${ROS_ROOT}/bags/${BAG_NAME}"
  LOG_FILE="${ROS_ROOT}/logs/bag_record_${BAG_NAME}.log"
  PID_FILE="${ROS_ROOT}/logs/bag_record_${BAG_NAME}.pid"
  UDP_BASELINE_FILE="${ROS_ROOT}/logs/bag_record_${BAG_NAME}.udp_start"
}

read_udp_receive_errors() {
  awk '
    $1 == "Udp:" && !header_seen {
      for (i = 2; i <= NF; i++) field[$i] = i
      header_seen = 1
      next
    }
    $1 == "Udp:" && header_seen {
      print $(field["InErrors"]), $(field["RcvbufErrors"])
      exit
    }
  ' /proc/net/snmp
}

pid_is_recorder() {
  local pid="$1"
  [[ "${pid}" =~ ^[0-9]+$ ]] \
    && kill -0 "${pid}" 2>/dev/null \
    && ps -p "${pid}" -o args= | grep -q "ros2 bag record"
}

read_pid() {
  if [[ -r "${PID_FILE}" ]]; then
    tr -d '[:space:]' < "${PID_FILE}"
  fi
}

prepare_bag_path() {
  if [[ ! -e "${BAG_PATH}" ]]; then
    return
  fi
  if [[ "${OVERWRITE_EXISTING_BAG}" != "true" ]]; then
    echo "Refusing to overwrite existing bag: ${BAG_PATH}" >&2
    echo "Use 'start-overwrite ${BAG_NAME}' to archive it and record a replacement." >&2
    exit 1
  fi

  local pid archive_root archive_path stamp process_matches
  pid="$(read_pid)"
  if pid_is_recorder "${pid}"; then
    echo "Refusing to overwrite a bag that is still being recorded: pid=${pid}" >&2
    exit 1
  fi
  process_matches="$(pgrep -af "ros2 bag record.*${BAG_PATH}" || true)"
  if [[ -n "${process_matches}" ]]; then
    echo "Refusing to overwrite because a recorder still references ${BAG_PATH}:" >&2
    printf '%s\n' "${process_matches}" >&2
    exit 1
  fi

  stamp="$(date +%Y%m%d_%H%M%S)"
  archive_root="${ROS_ROOT}/bags/overwritten"
  archive_path="${archive_root}/${BAG_NAME}_${stamp}"
  mkdir -p "${archive_root}"
  [[ ! -e "${archive_path}" ]] || {
    echo "Overwrite archive path already exists: ${archive_path}" >&2
    exit 1
  }
  mv "${BAG_PATH}" "${archive_path}"
  for path in \
    "${LOG_FILE}" \
    "${PID_FILE}" \
    "${UDP_BASELINE_FILE}"; do
    if [[ -e "${path}" ]]; then
      mv "${path}" "${archive_path}/$(basename "${path}")"
    fi
  done
  echo "ARCHIVED_EXISTING bag=${BAG_PATH} archive=${archive_path}"
}

wait_for_publisher() {
  local topic="$1"
  local info=""
  for _ in $(seq 1 15); do
    # Capture the complete output before matching it. With pipefail enabled,
    # piping ros2 topic info into grep -q can make ros2 hit EPIPE after grep
    # finds an early match and closes the pipe.
    info="$(timeout 5 "${ROS_ENV}" ros2 topic info "${topic}" 2>&1 || true)"
    if grep -Eq '^Publisher count: [1-9][0-9]*$' <<< "${info}"; then
      return 0
    fi
    sleep 1
  done
  echo "Last topic info for ${topic}:" >&2
  printf '%s\n' "${info}" >&2
  return 1
}

recorder_has_required_subscriptions() {
  local nodes node info
  if [[ -r "${LOG_FILE}" ]] \
    && grep -Fq "Subscribed to topic '/livox/lidar'" "${LOG_FILE}" \
    && grep -Fq "Subscribed to topic '/livox/imu'" "${LOG_FILE}"; then
    return 0
  fi

  nodes="$(timeout 5 "${ROS_ENV}" ros2 node list 2>/dev/null || true)"
  node="$(grep -E '^/.*rosbag2_recorder' <<< "${nodes}" | head -1 || true)"
  if [[ -n "${node}" ]]; then
    info="$(timeout 5 "${ROS_ENV}" ros2 node info "${node}" 2>&1 || true)"
    if grep -Fq '/livox/lidar:' <<< "${info}" \
      && grep -Fq '/livox/imu:' <<< "${info}"; then
      return 0
    fi
  fi

  # Discovery can lag even though rosbag2 has already confirmed both
  # subscriptions. Its own log is authoritative and avoids stopping a valid
  # recording merely because `ros2 node info` timed out.
  return 1
}

case "${ACTION}" in
  start)
    validate_name
    paths
    mkdir -p "${ROS_ROOT}/bags" "${ROS_ROOT}/logs"
    [[ -x "${ROS_ENV}" ]] || { echo "Missing ROS environment: ${ROS_ENV}" >&2; exit 1; }
    [[ -r "${QOS_FILE}" ]] || { echo "Missing QoS file: ${QOS_FILE}" >&2; exit 1; }
    if [[ -e "${BAG_PATH}" && "${OVERWRITE_EXISTING_BAG}" != "true" ]]; then
      echo "Refusing to overwrite existing bag: ${BAG_PATH}" >&2
      echo "Use 'start-overwrite ${BAG_NAME}' to archive it and record a replacement." >&2
      exit 1
    fi
    for topic in /livox/lidar /livox/imu; do
      if ! wait_for_publisher "${topic}"; then
        echo "No publisher for required topic: ${topic}" >&2
        exit 1
      fi
    done
    echo "Checking calibrated LiDAR mounting pose before recording..."
    "${ROS_ENV}" /usr/bin/python3 "${ROS_ROOT}/scripts/verify_live_livox_mount.py" \
      --expected-roll "${EXPECTED_LIDAR_ROLL_DEG}" \
      --expected-pitch "${EXPECTED_LIDAR_PITCH_DEG}" \
      --max-angle-error "${LIDAR_MOUNT_TOLERANCE_DEG}"
    prepare_bag_path
    read_udp_receive_errors > "${UDP_BASELINE_FILE}"
    : > "${LOG_FILE}"
    nohup "${ROS_ENV}" ros2 bag record \
      -s sqlite3 --max-cache-size "${CACHE_BYTES}" \
      --qos-profile-overrides-path "${QOS_FILE}" \
      -o "${BAG_PATH}" /livox/lidar /livox/imu \
      >>"${LOG_FILE}" 2>&1 </dev/null &
    pid=$!
    printf '%s\n' "${pid}" > "${PID_FILE}"
    for _ in $(seq 1 30); do
      if grep -q 'Recording...' "${LOG_FILE}" \
        && grep -q 'Opened database' "${LOG_FILE}" \
        && recorder_has_required_subscriptions; then
        echo "RECORDING bag=${BAG_PATH} pid=${pid} cache=${CACHE_BYTES}"
        echo "Required publishers and rosbag2 subscriptions are verified."
        echo "Keep the chassis completely still for the first 12 seconds."
        exit 0
      fi
      pid_is_recorder "${pid}" || {
        tail -n 100 "${LOG_FILE}" >&2 || true
        exit 1
      }
      sleep 1
    done
    echo "Recorder did not open its database and subscribe to both Livox topics within 30 seconds." >&2
    kill -INT "${pid}" 2>/dev/null || true
    exit 1
    ;;
  stop)
    validate_name
    paths
    pid="$(read_pid)"
    if ! pid_is_recorder "${pid}"; then
      if [[ ! -s "${BAG_PATH}/metadata.yaml" ]]; then
        echo "NOT_RUNNING: no recorder and no complete bag for ${BAG_NAME}." >&2
        exit 2
      fi
      echo "Recorder is not running for ${BAG_NAME}; validating existing bag." >&2
    else
      kill -INT "${pid}"
      for _ in $(seq 1 60); do
        kill -0 "${pid}" 2>/dev/null || break
        sleep 1
      done
      if kill -0 "${pid}" 2>/dev/null; then
        echo "Recorder did not stop within 60 seconds." >&2
        exit 1
      fi
    fi
    rm -f "${PID_FILE}"
    [[ -r "${LOG_FILE}" ]] || {
      echo "Missing recorder log: ${LOG_FILE}" >&2; exit 1; }
    grep -q 'Writing remaining messages from cache' "${LOG_FILE}" || {
      echo "Missing cache-flush confirmation in ${LOG_FILE}" >&2; exit 1; }
    grep -q 'Recording stopped' "${LOG_FILE}" || {
      echo "Missing recording-stopped confirmation in ${LOG_FILE}" >&2; exit 1; }
    [[ -s "${BAG_PATH}/metadata.yaml" ]] || {
      echo "Missing bag metadata: ${BAG_PATH}/metadata.yaml" >&2; exit 1; }
    udp_quality_ok=true
    if [[ -r "${UDP_BASELINE_FILE}" ]]; then
      read -r udp_in_start udp_buf_start < "${UDP_BASELINE_FILE}"
      read -r udp_in_end udp_buf_end <<< "$(read_udp_receive_errors)"
      udp_in_delta=$((udp_in_end - udp_in_start))
      udp_buf_delta=$((udp_buf_end - udp_buf_start))
      if (( udp_in_delta == 0 && udp_buf_delta == 0 )); then
        echo "UDP receive quality: PASS (counter deltas: in_errors=0 rcvbuf_errors=0)"
      else
        echo "UDP receive counter deltas: in_errors=${udp_in_delta} rcvbuf_errors=${udp_buf_delta}"
      fi
      if (( udp_buf_delta > 0 )); then
        echo "Kernel UDP receive-buffer errors increased during recording; bag is not production quality." >&2
        udp_quality_ok=false
      fi
    else
      echo "Missing UDP baseline: ${UDP_BASELINE_FILE}" >&2
      exit 4
    fi
    "${ROS_ENV}" ros2 bag info "${BAG_PATH}"
    [[ -r "${CONTINUITY_ANALYZER}" ]] || {
      echo "Missing bag continuity analyzer: ${CONTINUITY_ANALYZER}" >&2
      exit 5
    }
    echo "Running fast bag continuity validation; skipped checks below are informational, not errors."
    "${ROS_ENV}" /usr/bin/python3 "${CONTINUITY_ANALYZER}" "${BAG_PATH}"
    rm -f "${UDP_BASELINE_FILE}"
    if [[ "${udp_quality_ok}" != "true" ]]; then
      exit 4
    fi
    echo "RESULT: PASS bag flush, UDP receive quality, and topic continuity"
    echo "BAG_DIR=${BAG_PATH}"
    echo "Optional full mounting-pose validation:"
    printf '  %q /usr/bin/python3 %q %q --expected-roll %q --expected-pitch %q --max-angle-error 0.75\n' \
      "${ROS_ENV}" "${ROS_ROOT}/scripts/analyze_livox_bag_mount.py" \
      "${BAG_PATH}" "${EXPECTED_LIDAR_ROLL_DEG}" "${EXPECTED_LIDAR_PITCH_DEG}"
    ;;
  status)
    validate_name
    paths
    pid="$(read_pid)"
    if pid_is_recorder "${pid}"; then
      if recorder_has_required_subscriptions; then
        echo "RUNNING_SUBSCRIBED bag=${BAG_PATH} pid=${pid}"
      else
        echo "RUNNING_UNSUBSCRIBED bag=${BAG_PATH} pid=${pid}" >&2
        exit 3
      fi
    elif [[ -s "${BAG_PATH}/metadata.yaml" ]]; then
      echo "STOPPED_COMPLETE bag=${BAG_PATH}"
    else
      echo "NOT_RUNNING bag=${BAG_PATH}"
      exit 2
    fi
    ;;
  *)
    echo "Usage: $0 {start|start-overwrite|stop|status} BAG_NAME" >&2
    exit 1
    ;;
esac
