#!/usr/bin/env bash
#
# distclean.sh
#

. ./0-common.sh

rm -rf build log install
rm -rf "$SRCDIR"/xarm_ros2
