#!/usr/bin/env bash
#
# 0-common.sh
#

set -e
set -o pipefail
# set -u
# set -x

umask 0022

################################################################################

. /opt/ros/humble/setup.bash

SRCDIR="../src/xarm_humble"
DEP_PKGS_FILE="dep-pkgs.txt"
DEP_PKGS_TO_INSTALL_FILE="dep-pkgs-to-install.txt"

update_src() {
    if [[ ! -d "$SRCDIR" ]]; then
        mkdir -p "$SRCDIR"
    fi
    vcs import --force --recursive --input pkgs.repos "$SRCDIR"
}

update_deps() {
    # rosdep update --rosdistro=$ROS_DISTRO
    rosdep install \
        --rosdistro="$ROS_DISTRO" \
        --reinstall \
        --from-paths "$SRCDIR" \
        --ignore-src \
        -s | awk '{print $5}' | sed -E -e '/^\s*$/d' -e "s/'$//g" | LC_ALL=C sort -n > "$DEP_PKGS_FILE"

    sed -i \
        -e '/^cmake$/d' \
        -e '/^libboost-/ {/libboost-all-dev/!d}' \
        -e '/^pkg-config$/d' \
        "$DEP_PKGS_FILE"

    cat "$DEP_PKGS_FILE" | xargs sudo apt-get install --no-install-recommends -s \
        | (grep "^Inst" || :) | awk '{print $2}' | LC_ALL=C sort -n \
        > "$DEP_PKGS_TO_INSTALL_FILE"
}

install_deps() {
    if [[ -s "$DEP_PKGS_FILE" ]]; then
        cat "$DEP_PKGS_FILE"  | xargs sudo apt-get install --no-install-recommends -y
    fi
}
