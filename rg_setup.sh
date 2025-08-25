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

################
. "$RG_WS/../../ros2_ws/ros2_setup.sh"
. "$RG_WS/../../ros2_ws/nav2_setup.sh"
. "$RG_WS/../../ros2_ws/moveit2_setup.sh"

################
. "$RG_WS/../../IsaacSim-ros_workspaces/setup.sh"

################
RG_SETUP="$RG_WS/install/local_setup.$shell"
if [[ -f "$RG_SETUP" ]]; then
    echo "rg_ws ($shell)"
    . "$RG_SETUP"
else
    echo "failed to set up rg_ws"
    exit 1
fi

################
export WAYLAND_DISPLAY="wayland-0"
export DISPLAY=":0"
