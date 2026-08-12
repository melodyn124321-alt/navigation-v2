#!/usr/bin/env bash
set -euo pipefail

ROS_ROOT="${ROS_ROOT:-/home/seeed/ros2}"
ROS_ENV="${ROS_ENV:-${ROS_ROOT}/use_ros_env.sh}"
BAG_PATH="${1:-${ROS_ROOT}/bags/raw_livox_20260703_124600}"
OUT_DIR="${2:-${ROS_ROOT}/maps/replay}"
OUTPUT_PCD="${3:-${OUTPUT_PCD:-}}"
RATE="${RATE:-0.5}"
CLOCK_HZ="${CLOCK_HZ:-100}"
HN_RVIZ_WAIT_SEC="${HN_RVIZ_WAIT_SEC:-8}"
ODOM_ABORT_METERS="${ODOM_ABORT_METERS:-100}"
MIN_PCD_BYTES="${MIN_PCD_BYTES:-1000000}"
FASTLIO_ABORT_ON_OVERFLOW="${FASTLIO_ABORT_ON_OVERFLOW:-true}"
FASTLIO_NO_EFFECTIVE_ABORT_COUNT="${FASTLIO_NO_EFFECTIVE_ABORT_COUNT:-80}"
BAG_PREFLIGHT="${BAG_PREFLIGHT:-${ROS_ROOT}/scripts/analyze_livox_bag_mount.py}"
REQUIRE_BAG_PREFLIGHT="${REQUIRE_BAG_PREFLIGHT:-true}"
ALLOW_UNSAFE_BAG_PREFLIGHT="${ALLOW_UNSAFE_BAG_PREFLIGHT:-false}"
EXPECTED_LIDAR_ROLL_DEG="${EXPECTED_LIDAR_ROLL_DEG:--0.003}"
EXPECTED_LIDAR_PITCH_DEG="${EXPECTED_LIDAR_PITCH_DEG:-2.258}"
LIDAR_MOUNT_TOLERANCE_DEG="${LIDAR_MOUNT_TOLERANCE_DEG:-0.75}"
SELF_RETURN_ANALYZER="${SELF_RETURN_ANALYZER:-${ROS_ROOT}/scripts/analyze_livox_self_returns.py}"
REQUIRE_SELF_RETURN_CHECK="${REQUIRE_SELF_RETURN_CHECK:-true}"
MAP_CLEAN_ENABLE="${MAP_CLEAN_ENABLE:-true}"
MAP_VOXEL_LEAF="${MAP_VOXEL_LEAF:-0.06}"
MAP_OUTLIER_RADIUS="${MAP_OUTLIER_RADIUS:-0.25}"
MAP_OUTLIER_MIN_PTS="${MAP_OUTLIER_MIN_PTS:-2}"
FASTLIO_PID=""
BAG_PID=""

cleanup() {
  if [ -n "${BAG_PID}" ]; then
    kill -INT "${BAG_PID}" 2>/dev/null || true
    wait "${BAG_PID}" 2>/dev/null || true
  fi
  if [ -n "${FASTLIO_PID}" ]; then
    kill "${FASTLIO_PID}" 2>/dev/null || true
    wait "${FASTLIO_PID}" 2>/dev/null || true
  fi
  pkill -x fastlio_mapping 2>/dev/null || true
}
trap cleanup EXIT

check_fastlio_divergence() {
  local log_file="$1"
  if [ ! -f "${log_file}" ]; then
    return 0
  fi

  if [ "${FASTLIO_ABORT_ON_OVERFLOW}" = "true" ] && \
     grep -q "Integer indices would overflow" "${log_file}"; then
    echo "ERROR: Fast-LIO/PCL voxel grid overflow detected. The replay has diverged; abort without saving PCD." >&2
    echo "Matched log lines:" >&2
    grep "Integer indices would overflow" "${log_file}" | tail -10 >&2
    return 1
  fi

  local no_effective_count
  no_effective_count="$(tail -300 "${log_file}" | grep -c "No Effective Points" || true)"
  if [ "${no_effective_count}" -ge "${FASTLIO_NO_EFFECTIVE_ABORT_COUNT}" ]; then
    echo "ERROR: Fast-LIO has ${no_effective_count} 'No Effective Points' messages in recent logs. The replay is likely diverging or the LiDAR data is unusable; abort without saving PCD." >&2
    return 1
  fi
}

if [ ! -x "${ROS_ENV}" ]; then
  echo "ROS env script not executable: ${ROS_ENV}" >&2
  exit 1
fi

if [ ! -f "${BAG_PATH}/metadata.yaml" ]; then
  echo "Bag metadata not found: ${BAG_PATH}/metadata.yaml" >&2
  echo "Use the bag directory, not a .db3 file and not an unset variable." >&2
  exit 1
fi

mkdir -p "${OUT_DIR}" "${ROS_ROOT}/logs"

BAG_NAME="$(basename "${BAG_PATH}")"
STAMP="$(date +%Y%m%d_%H%M%S)"
if [ -n "${OUTPUT_PCD}" ]; then
  case "${OUTPUT_PCD}" in
    /*) MAP_PATH="${OUTPUT_PCD}" ;;
    *) MAP_PATH="${OUT_DIR}/${OUTPUT_PCD}" ;;
  esac
else
  MAP_PATH="${OUT_DIR}/fastlio_map_from_${BAG_NAME}_${STAMP}.pcd"
fi
mkdir -p "$(dirname "${MAP_PATH}")"
CONFIG_DIR="/tmp/fastlio_replay_${BAG_NAME}_${STAMP}"
CONFIG_FILE="mid360_replay.yaml"
CONFIG_PATH="${CONFIG_DIR}/${CONFIG_FILE}"

echo "Bag: ${BAG_PATH}"
echo "Output PCD: ${MAP_PATH}"
echo "Replay rate: ${RATE}"
echo "Clock rate: ${CLOCK_HZ} Hz"
echo "Abort on voxel overflow: ${FASTLIO_ABORT_ON_OVERFLOW}"
echo "Abort on recent No Effective Points count >= ${FASTLIO_NO_EFFECTIVE_ABORT_COUNT}"
echo "Static-map cleanup: ${MAP_CLEAN_ENABLE} (voxel=${MAP_VOXEL_LEAF}m radius=${MAP_OUTLIER_RADIUS}m min_pts=${MAP_OUTLIER_MIN_PTS})"

if [ "${REQUIRE_SELF_RETURN_CHECK}" = "true" ]; then
  if [ ! -r "${SELF_RETURN_ANALYZER}" ]; then
    echo "Required self-return analyzer is missing: ${SELF_RETURN_ANALYZER}" >&2
    exit 6
  fi
  echo "Checking that the electrical-cabinet return is covered by the range gate..."
  "${ROS_ENV}" python3 "${SELF_RETURN_ANALYZER}" "${BAG_PATH}" \
    --sample-frames 300 --max-range 1.50 --voxel 0.04 \
    --min-persistence 0.03 --mapping-blind 0.70 --check-range-gate
fi

if [ "${REQUIRE_BAG_PREFLIGHT}" = "true" ]; then
  if [ ! -r "${BAG_PREFLIGHT}" ]; then
    echo "Required bag preflight tool is missing: ${BAG_PREFLIGHT}" >&2
    exit 5
  fi
  echo "Running bag continuity and mounting-angle preflight..."
  if ! "${ROS_ENV}" python3 "${BAG_PREFLIGHT}" "${BAG_PATH}" \
    --expected-roll "${EXPECTED_LIDAR_ROLL_DEG}" \
    --expected-pitch "${EXPECTED_LIDAR_PITCH_DEG}" \
    --max-angle-error "${LIDAR_MOUNT_TOLERANCE_DEG}"; then
    if [ "${ALLOW_UNSAFE_BAG_PREFLIGHT}" != "true" ]; then
      echo "ERROR: bag preflight failed. Refusing to create a production PCD." >&2
      echo "For diagnostic conversion only, explicitly set ALLOW_UNSAFE_BAG_PREFLIGHT=true." >&2
      exit 5
    fi
    echo "WARNING: unsafe bag preflight override enabled; output is diagnostic only." >&2
  fi
fi

"${ROS_ROOT}/scripts/seeed_stop_ros_stack.sh"

mkdir -p "${CONFIG_DIR}"
cp "${ROS_ROOT}/ws/install/fast_lio/share/fast_lio/config/mid360.yaml" "${CONFIG_PATH}"
python3 - "$CONFIG_PATH" "$MAP_PATH" <<'PY'
import re
import sys
cfg, pcd = sys.argv[1], sys.argv[2]
text = open(cfg, "r", encoding="utf-8").read()
text = re.sub(r'map_file_path:\s*"[^"]*"', f'map_file_path: "{pcd}"', text)
text = re.sub(r'map_en:\s*(true|false)', 'map_en: true', text)
text = re.sub(r'dense_publish_en:\s*(true|false)', 'dense_publish_en: true', text)
text = re.sub(r'effect_map_en:\s*(true|false)', 'effect_map_en: false', text)
text = re.sub(r'pcd_save_en:\s*(true|false)', 'pcd_save_en: true', text)
open(cfg, "w", encoding="utf-8").write(text)
PY

"${ROS_ENV}" ros2 bag info "${BAG_PATH}" | tee "${ROS_ROOT}/logs/${BAG_NAME}_${STAMP}_bag_info.log"

echo "Starting Fast-LIO for offline replay..."
"${ROS_ENV}" ros2 launch fast_lio mapping.launch.py \
  rviz:=false \
  use_sim_time:=true \
  config_path:="${CONFIG_DIR}" \
  config_file:="${CONFIG_FILE}" \
  > "${ROS_ROOT}/logs/fastlio_replay_${BAG_NAME}_${STAMP}.log" 2>&1 &
FASTLIO_PID=$!

echo "Waiting for /map_save service..."
MAP_SAVE_READY=false
for _ in $(seq 1 30); do
  if ! kill -0 "${FASTLIO_PID}" 2>/dev/null; then
    echo "Fast-LIO exited before /map_save became available. See log:" >&2
    tail -80 "${ROS_ROOT}/logs/fastlio_replay_${BAG_NAME}_${STAMP}.log" >&2
    exit 1
  fi
  if "${ROS_ENV}" ros2 service list 2>/dev/null | grep -q "^/map_save$"; then
    MAP_SAVE_READY=true
    break
  fi
  sleep 1
done

if [ "${MAP_SAVE_READY}" != "true" ]; then
  echo "/map_save service is not available. See log:" >&2
  tail -80 "${ROS_ROOT}/logs/fastlio_replay_${BAG_NAME}_${STAMP}.log" >&2
  kill "${FASTLIO_PID}" 2>/dev/null || true
  exit 1
fi

echo "Fast-LIO replay node is ready. On hn, run:"
echo "  /home/hn/ros_hn/scripts/hn_watch_bag_mapping.sh"
echo "Waiting ${HN_RVIZ_WAIT_SEC}s for hn RViz DDS discovery before bag playback..."
sleep "${HN_RVIZ_WAIT_SEC}"

echo "Playing bag. Keep hn RViz open with fastlio_bag_mapping.rviz to watch replay mapping."
echo "Progress is printed every 15 seconds; do not move the live LiDAR for this offline replay."
"${ROS_ENV}" ros2 bag play "${BAG_PATH}" \
  --clock "${CLOCK_HZ}" \
  --disable-keyboard-controls \
  --topics /livox/lidar /livox/imu \
  -r "${RATE}" \
  > "${ROS_ROOT}/logs/bag_play_${BAG_NAME}_${STAMP}.log" 2>&1 &
BAG_PID=$!

while kill -0 "${BAG_PID}" 2>/dev/null; do
  sleep 15
  if ! kill -0 "${BAG_PID}" 2>/dev/null; then
    break
  fi
  echo "--- replay still running: $(date '+%F %T') ---"
  ps -p "${BAG_PID}" -o pid,etime,comm,args || true
  ps -p "${FASTLIO_PID}" -o pid,etime,comm,args || true
  "${ROS_ENV}" ros2 topic list 2>/dev/null | egrep "^/clock$|^/Odometry$|^/path$|^/cloud_registered$|^/cloud_registered_body$|^/tf$" || true
  for topic in /clock /livox/lidar /cloud_registered /Odometry; do
    echo "Topic hz check: ${topic}"
    timeout 5 "${ROS_ENV}" ros2 topic hz "${topic}" --window 5 2>/dev/null | sed -n '1,3p' || true
  done
  check_fastlio_divergence "${ROS_ROOT}/logs/fastlio_replay_${BAG_NAME}_${STAMP}.log" || {
    kill -INT "${BAG_PID}" 2>/dev/null || true
    wait "${BAG_PID}" 2>/dev/null || true
    BAG_PID=""
    exit 4
  }
  ODOM_SAMPLE="$(timeout 3 "${ROS_ENV}" ros2 topic echo --once /Odometry 2>/dev/null || true)"
  ODOM_XYZ="$(printf "%s\n" "${ODOM_SAMPLE}" | awk '
    /position:/ {in_pos=1; next}
    in_pos && /x:/ {x=$2; next}
    in_pos && /y:/ {y=$2; next}
    in_pos && /z:/ {z=$2; print x, y, z; exit}
  ')"
  if [ -n "${ODOM_XYZ}" ]; then
    echo "Odometry position xyz: ${ODOM_XYZ}"
    if ! python3 - "${ODOM_ABORT_METERS}" ${ODOM_XYZ} <<'PY'
import sys
limit = float(sys.argv[1])
xyz = [float(v) for v in sys.argv[2:5]]
sys.exit(0 if max(abs(v) for v in xyz) <= limit else 1)
PY
    then
      echo "ERROR: Odometry exceeded ${ODOM_ABORT_METERS} m during replay; Fast-LIO is diverging. Abort without saving PCD." >&2
      kill -INT "${BAG_PID}" 2>/dev/null || true
      wait "${BAG_PID}" 2>/dev/null || true
      BAG_PID=""
      exit 3
    fi
  else
    echo "Odometry position xyz: unavailable yet"
  fi
  tail -5 "${ROS_ROOT}/logs/fastlio_replay_${BAG_NAME}_${STAMP}.log" || true
done

wait "${BAG_PID}"
BAG_PID=""

echo "Bag playback ended. Saving PCD..."
sleep 5
check_fastlio_divergence "${ROS_ROOT}/logs/fastlio_replay_${BAG_NAME}_${STAMP}.log" || exit 4
"${ROS_ENV}" ros2 service list | grep -q "^/map_save$" || {
  echo "/map_save service disappeared before saving. Fast-LIO probably exited or crashed." >&2
  echo "Fast-LIO log:" >&2
  tail -100 "${ROS_ROOT}/logs/fastlio_replay_${BAG_NAME}_${STAMP}.log" >&2
  exit 1
}
"${ROS_ENV}" ros2 service call /map_save std_srvs/srv/Trigger "{}" \
  | tee "${ROS_ROOT}/logs/map_save_${BAG_NAME}_${STAMP}.log"

sleep 3
cleanup
FASTLIO_PID=""

if [ ! -s "${MAP_PATH}" ]; then
  echo "PCD was not created or is empty: ${MAP_PATH}" >&2
  exit 1
fi

if [ "${MAP_CLEAN_ENABLE}" = "true" ]; then
  for tool in pcl_voxel_grid pcl_outlier_removal pcl_convert_pcd_ascii_binary; do
    command -v "${tool}" >/dev/null || {
      echo "${tool} is required for static-map cleanup." >&2
      exit 1
    }
  done

  RAW_MAP_PATH="${MAP_PATH%.pcd}_raw.pcd"
  VOXEL_MAP_PATH="${MAP_PATH%.pcd}_voxel_tmp.pcd"
  COMPRESSED_MAP_PATH="${MAP_PATH%.pcd}_compressed_tmp.pcd"
  BINARY_MAP_PATH="${MAP_PATH%.pcd}_binary_tmp.pcd"
  VALIDATION_MAP_PATH="${MAP_PATH%.pcd}_validation_tmp.pcd"
  mv -f "${MAP_PATH}" "${RAW_MAP_PATH}"
  pcl_voxel_grid "${RAW_MAP_PATH}" "${VOXEL_MAP_PATH}" \
    -leaf "${MAP_VOXEL_LEAF},${MAP_VOXEL_LEAF},${MAP_VOXEL_LEAF}"
  pcl_outlier_removal "${VOXEL_MAP_PATH}" "${COMPRESSED_MAP_PATH}" \
    -method radius -radius "${MAP_OUTLIER_RADIUS}" \
    -min_pts "${MAP_OUTLIER_MIN_PTS}"
  pcl_convert_pcd_ascii_binary "${COMPRESSED_MAP_PATH}" "${BINARY_MAP_PATH}" 1
  pcl_convert_pcd_ascii_binary "${BINARY_MAP_PATH}" "${VALIDATION_MAP_PATH}" 1
  if [ ! -s "${BINARY_MAP_PATH}" ] || [ ! -s "${VALIDATION_MAP_PATH}" ]; then
    echo "Clean PCD validation failed; raw map retained at ${RAW_MAP_PATH}." >&2
    exit 1
  fi
  mv -f "${BINARY_MAP_PATH}" "${MAP_PATH}"
  rm -f "${VOXEL_MAP_PATH}" "${COMPRESSED_MAP_PATH}" "${VALIDATION_MAP_PATH}"
  echo "Raw map retained: ${RAW_MAP_PATH}"
  echo "Clean NDT-compatible binary map: ${MAP_PATH}"
fi

if [ ! -s "${MAP_PATH}" ]; then
  echo "Final PCD is empty after cleanup: ${MAP_PATH}" >&2
  exit 1
fi

SIZE="$(stat -c%s "${MAP_PATH}")"
echo "${MAP_PATH}" > "${OUT_DIR}/latest_${BAG_NAME}_target.txt"
echo "Saved PCD size: ${SIZE} bytes"
echo "Latest target file: ${OUT_DIR}/latest_${BAG_NAME}_target.txt"
if [ "${SIZE}" -lt "${MIN_PCD_BYTES}" ]; then
  echo "WARNING: PCD is smaller than ${MIN_PCD_BYTES} bytes. This usually means Fast-LIO did not initialize, the replay was interrupted, or the bag is only a short smoke test." >&2
  exit 2
fi
