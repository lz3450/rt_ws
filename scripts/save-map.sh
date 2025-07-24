#!/bin/bash

if [[ -z "$ROS_DISTRO" ]]; then
    . ../robotrace_setup.sh
fi

ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
"name:
  data: 'ecss3220'"

mv ecss3220.{pgm,yaml} ../maps
