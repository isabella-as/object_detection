#!/bin/bash
set -e

unset ROS_DISTRO
source "/opt/ros/noetic/setup.bash"

roscore & sleep 2
rosparam load /bridge.yaml

unset ROS_DISTRO
source "/opt/ros/foxy/setup.bash"

export ROS_MASTER_URI=http://localhost:11311
ros2 run ros1_bridge parameter_bridge

exec "$@"
