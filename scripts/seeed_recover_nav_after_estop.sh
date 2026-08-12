#!/usr/bin/env bash
set -euo pipefail

ROS_ROOT="${ROS_ROOT:-/home/seeed/ros2}"
ROS_ENV="${ROS_ENV:-${ROS_ROOT}/use_ros_env.sh}"
CANCEL_SCRIPT="${CANCEL_SCRIPT:-${ROS_ROOT}/scripts/seeed_cancel_nav_goal_safe.sh}"
RECOVERY_CHECK="${RECOVERY_CHECK:-${ROS_ROOT}/scripts/nav_estop_recovery_check.py}"
ARM_SCRIPT="${ARM_SCRIPT:-${ROS_ROOT}/scripts/seeed_arm_nav_motion.sh}"
NAV_FASTDDS_PROFILE="${NAV_FASTDDS_PROFILE:-${ROS_ROOT}/fastrtps_profile.xml}"
RECOVER_MOTION="${RECOVER_MOTION:-false}"
CONFIRM_RECOVERY="${CONFIRM_RECOVERY:-}"

for path in "${CANCEL_SCRIPT}" "${RECOVERY_CHECK}" "${ARM_SCRIPT}"; do
  if [[ ! -x "${path}" ]]; then
    echo "Required recovery file is missing or not executable: ${path}" >&2
    exit 1
  fi
done

echo "[1/4] Canceling every old goal and forcing motion DISARMED..."
"${CANCEL_SCRIPT}"

echo "[2/4] Waiting for stable sonar, LiDAR/localization, chassis and zero velocity..."
timeout 28 "${ROS_ENV}" env \
  FASTRTPS_DEFAULT_PROFILES_FILE="${NAV_FASTDDS_PROFILE}" \
  ROS2CLI_DISABLE_DAEMON=1 \
  python3 "${RECOVERY_CHECK}" \
  --timeout 22 \
  --max-fitness 0.10 \
  --required-samples 4

echo "[3/4] Running the complete arm-readiness check without arming..."
DRY_RUN=true "${ARM_SCRIPT}"

if [[ "${RECOVER_MOTION}" != "true" ]]; then
  echo "ESTOP_RECOVERY_READY motion remains DISARMED."
  echo "After visually confirming the cleared area, explicitly run:"
  echo "  RECOVER_MOTION=true CONFIRM_RECOVERY=I_UNDERSTAND $0"
  exit 0
fi
if [[ "${CONFIRM_RECOVERY}" != "I_UNDERSTAND" ]]; then
  echo "Refusing to re-arm without CONFIRM_RECOVERY=I_UNDERSTAND." >&2
  exit 2
fi

echo "[4/4] Re-running readiness and explicitly arming motion..."
"${ARM_SCRIPT}"
echo "ESTOP_RECOVERY_ARMED"
echo "The canceled goal was not resumed. Send one new 2D Goal Pose in RViz."
