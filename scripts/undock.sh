#!/bin/bash

if [[ -z "$ROS_DISTRO" ]]; then
    . ../robotrace_setup.sh
fi

ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
