#!/usr/bin/env bash
set -euo pipefail

ROS_ROOT="${ROS_ROOT:-/home/seeed/ros2}"
PCD_PATH="${1:-${ROS_ROOT}/maps/replay/fastlio_map_manual_001_level_groundsafe_20260729.pcd}"
NAV_YAML="${2:-${ROS_ROOT}/maps/replay/fastlio_map_manual_001_level_groundsafe_20260729_nav.yaml}"

RESOLUTION="${RESOLUTION:-0.05}"
MIN_Z="${MIN_Z:-}"
MAX_Z="${MAX_Z:-}"
GROUND_CLEARANCE_M="${GROUND_CLEARANCE_M:-0.12}"
MAX_OBSTACLE_HEIGHT_M="${MAX_OBSTACLE_HEIGHT_M:-1.60}"
INFLATE_CELLS="${INFLATE_CELLS:-0}"
MIN_POINTS_PER_CELL="${MIN_POINTS_PER_CELL:-3}"
MIN_Z_SPAN="${MIN_Z_SPAN:-0.10}"
FLOOR_ESTIMATOR="${FLOOR_ESTIMATOR:-${ROS_ROOT}/scripts/estimate_pcd_floor.py}"
ROS_ENV="${ROS_ENV:-${ROS_ROOT}/use_ros_env.sh}"

if [[ ! -f "${PCD_PATH}" ]]; then
  echo "PCD not found: ${PCD_PATH}" >&2
  exit 1
fi

mkdir -p "$(dirname "${NAV_YAML}")"

if [[ -z "${MIN_Z}" || -z "${MAX_Z}" ]]; then
  if [[ ! -r "${FLOOR_ESTIMATOR}" || ! -x "${ROS_ENV}" ]]; then
    echo "Cannot derive a ground-relative height window: missing ${FLOOR_ESTIMATOR} or ${ROS_ENV}" >&2
    exit 2
  fi
  FLOOR_LOG="${ROS_ROOT}/logs/$(basename "${PCD_PATH%.pcd}")_floor_estimate.log"
  mkdir -p "${ROS_ROOT}/logs"
  "${ROS_ENV}" python3 "${FLOOR_ESTIMATOR}" "${PCD_PATH}" \
    --reference-normal 0 0 1 \
    --max-normal-angle-deg 5.0 | tee "${FLOOR_LOG}"
  FLOOR_Z="$(awk -F= '/^RESULT_FLOOR_Z=/{print $2}' "${FLOOR_LOG}" | tail -1)"
  if [[ -z "${FLOOR_Z}" ]]; then
    echo "Floor estimator did not return RESULT_FLOOR_Z." >&2
    exit 2
  fi
  MIN_Z="$(awk -v floor="${FLOOR_Z}" -v clearance="${GROUND_CLEARANCE_M}" \
    'BEGIN {printf "%.9f", floor + clearance}')"
  MAX_Z="$(awk -v floor="${FLOOR_Z}" -v height="${MAX_OBSTACLE_HEIGHT_M}" \
    'BEGIN {printf "%.9f", floor + height}')"
fi

echo "Preparing Nav2 occupancy map from PCD"
echo "  PCD: ${PCD_PATH}"
echo "  YAML: ${NAV_YAML}"
echo "  PGM: ${NAV_YAML}.pgm"
echo "  ground-relative z range: [${MIN_Z}, ${MAX_Z}]"

RESOLUTION="${RESOLUTION}" \
MIN_Z="${MIN_Z}" \
MAX_Z="${MAX_Z}" \
INFLATE_CELLS="${INFLATE_CELLS}" \
MIN_POINTS_PER_CELL="${MIN_POINTS_PER_CELL}" \
MIN_Z_SPAN="${MIN_Z_SPAN}" \
"${ROS_ROOT}/scripts/make_corrected_grid_from_pcd.sh" "${PCD_PATH}" "${NAV_YAML}"

echo "Map files:"
ls -lh "${NAV_YAML}" "${NAV_YAML}.pgm"
echo "--- YAML"
cat "${NAV_YAML}"
