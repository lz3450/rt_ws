#!/bin/bash

pids=()
LOG_DIR="log"

sigint_handler() {
    echo "Ctrl-C pressed! Sending SIGINT to all background processes..."
    for pid in "${pids[@]}"; do
        kill -INT "$pid"
    done
    # Optionally wait for all background jobs to exit
    wait
    exit 1
}
trap sigint_handler INT

if [[ -z "$ROS_DISTRO" ]]; then
    . ../robotrace_setup.sh
fi

mkdir -p "$LOG_DIR"

ros2 launch rt_tb4_navigation slam.launch.py > "$LOG_DIR"/slam.log &
pids+=($!)
sleep 2
# ros2 launch turtlebot4_navigation localization.launch.py map:=$HOME/ecss3220.yaml &
# pids+=($!)
# sleep 1
ros2 launch rt_tb4_navigation nav2.launch.py > "$LOG_DIR"/nav2.log &
pids+=($!)
sleep 3
ros2 launch turtlebot4_viz view_robot.launch.py > "$LOG_DIR"/view.log &
pids+=($!)

wait
