#!/usr/bin/env bash
#
# xarm_moveit.sh
#

. ~/Projects/RoboGuard/rg_ws/rg_setup.sh

ros2 launch xarm_moveit_config lite6_moveit_realmove.launch.py robot_ip:=192.168.1.179 add_gripper:=true
