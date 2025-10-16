#!/usr/bin/env bash
#
# 1-update-src.sh
#

. ./0-common.sh

UNUSED_PKGS=(
    "xarm_ros2/xarm_gazebo"
    "xarm_ros2/thirdparty/realsense_gazebo_plugin"
)

update_src

if [[ -d "patches" ]]; then
    for patch in $(find patches -type f -name "*.patch"); do
        repo=$(basename "$patch" .patch)
        rel_patch=$(realpath --relative-to="$SRCDIR/$repo" "$patch")
        echo "Applying patch: $patch to $SRCDIR/$repo"
        git -C "$SRCDIR/$repo" apply "$rel_patch"
    done
    unset patch repo
fi

for pkg in "${UNUSED_PKGS[@]}"; do
    if [[ -d "$SRCDIR/$pkg" ]]; then
        echo "Removing unused package: $SRCDIR/$pkg"
        rm -rf "$SRCDIR/$pkg"
    else
        echo "Package $pkg not found, skipping."
    fi
done

echo "Successfully updated xArm source code and applied patches"
