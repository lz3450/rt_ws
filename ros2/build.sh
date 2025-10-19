#!/bin/bash

set -e

if [[ -z $ROS_DISTRO ]]; then
    . /opt/ros/humble/setup.bash
fi

export MAKEFLAGS="-j $(nproc)"

COMMON_OPTIONS=(
    --symlink-install
    --merge-install
    --parallel-workers $(nproc)
    # --continue-on-error
    --packages-skip-build-finished
    --cmake-args
    -Wno-dev
    "-DCMAKE_BUILD_TYPE=Release"
    "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
    # "-DCMAKE_VERBOSE_MAKEFILE=ON"
    # "-DCMAKE_C_COMPILER=/opt/bin/clang"
    # "-DCMAKE_CXX_COMPILER=/opt/bin/clang++"
)

colcon build "${COMMON_OPTIONS[@]}"

# colcon build \
#     --build-base build-merge \
#     --install-base install-merge \
#     --merge-install \
#     "${COMMON_OPTIONS[@]}"
