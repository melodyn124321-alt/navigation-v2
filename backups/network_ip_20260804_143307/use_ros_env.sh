#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source /home/seeed/ros2/ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_IP=192.168.43.139
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/seeed/ros2/fastrtps_profile.xml
unset CYCLONEDDS_URI
unset PYTHONHOME
case "${1:-}" in
  python|python3)
    shift
    exec /usr/bin/python3 "$@"
    ;;
esac
exec "$@"
