if [ -n "$BASH_VERSION" ]; then
    export RG_WS="$(cd -- "$(dirname "${BASH_SOURCE[0]}")" > /dev/null 2>&1; pwd -P)"
        shell="bash"
elif [ -n "$ZSH_VERSION" ]; then
    export RG_WS="$(cd -- "$(dirname "${(%):-%x}")" > /dev/null 2>&1; pwd -P)"
        shell="zsh"
else
    echo "Unsupported shell"
fi
echo "RG_WS=$RG_WS"

################################################################################
export ROS_DOMAIN_ID=76
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
# export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"

################################################################################
if [ -f "/opt/ros/humble/setup.$shell" ]; then
    . /opt/ros/humble/setup.$shell
else
    echo "failed to set up ros2 humble"
    exit 1
fi

################################################################################
RG_SETUP="$RG_WS/ros2/install/local_setup.$shell"
if [[ -f "$RG_SETUP" ]]; then
    echo "rg_ws ($shell)"
    . "$RG_SETUP"
else
    echo "failed to set up rg_ws"
    exit 1
fi

################################################################################
export WAYLAND_DISPLAY="wayland-0"
export DISPLAY=":0"
