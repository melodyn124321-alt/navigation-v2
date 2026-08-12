#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

PCD_PATH="${1:-${ROS_ROOT}/maps/fastlio_map_from_raw_livox_094631_20260629_133011.pcd}"
OUT_YAML="${2:-${ROS_ROOT}/maps/fastlio_map_from_raw_livox_094631_corrected_grid.yaml}"

RESOLUTION="${RESOLUTION:-0.10}"
MIN_Z="${MIN_Z:-0.15}"
MAX_Z="${MAX_Z:-1.20}"
INFLATE_CELLS="${INFLATE_CELLS:-0}"
MIN_POINTS_PER_CELL="${MIN_POINTS_PER_CELL:-3}"
MIN_Z_SPAN="${MIN_Z_SPAN:-0.15}"

if [[ ! -f "$PCD_PATH" ]]; then
  echo "PCD not found: $PCD_PATH" >&2
  exit 1
fi

ENV_SCRIPT="${ROS_ROOT}/use_ros_env.sh"
if [[ ! -x "$ENV_SCRIPT" ]]; then
  ENV_SCRIPT="${ROS_ROOT}/use_fastdds_env.sh"
fi
if [[ ! -x "$ENV_SCRIPT" ]]; then
  echo "ROS environment script not found under: $ROS_ROOT" >&2
  exit 1
fi

mkdir -p "$(dirname "$OUT_YAML")"

echo "Generating corrected occupancy grid"
echo "  PCD: $PCD_PATH"
echo "  YAML: $OUT_YAML"
echo "  PGM: ${OUT_YAML}.pgm"
echo "  resolution=$RESOLUTION z=[$MIN_Z,$MAX_Z] inflate=$INFLATE_CELLS min_points=$MIN_POINTS_PER_CELL min_z_span=$MIN_Z_SPAN"

timeout 8s "$ENV_SCRIPT" \
  ros2 run pcd_ndt_localization pcd_occupancy_grid_publisher \
  --ros-args \
  -p map_path:="$PCD_PATH" \
  -p map_yaml_path:="$OUT_YAML" \
  -p resolution:="$RESOLUTION" \
  -p min_z:="$MIN_Z" \
  -p max_z:="$MAX_Z" \
  -p inflate_cells:="$INFLATE_CELLS" \
  -p min_points_per_cell:="$MIN_POINTS_PER_CELL" \
  -p min_z_span:="$MIN_Z_SPAN" \
  -p save_map_files:=true \
  -p publish_period_sec:=1.0 || true

if [[ ! -s "$OUT_YAML" || ! -s "${OUT_YAML}.pgm" ]]; then
  echo "Failed to generate occupancy grid files" >&2
  exit 1
fi

ls -lh "$OUT_YAML" "${OUT_YAML}.pgm"
