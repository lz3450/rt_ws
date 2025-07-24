#!/bin/bash

if [[ -z "$ROS_DISTRO" ]]; then
    . ../robotrace_setup.sh
fi

ros2 action send_goal /dock irobot_create_msgs/action/Dock "{}"
