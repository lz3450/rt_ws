if [ -n "$BASH_VERSION" ]; then
    export RT_WS="$(cd -- "$(dirname "${BASH_SOURCE[0]}")" > /dev/null 2>&1; pwd -P)"
    shell="bash"
elif [ -n "$ZSH_VERSION" ]; then
    export RT_WS="$(cd -- "$(dirname "${(%):-%x}")" > /dev/null 2>&1; pwd -P)"
    shell="zsh"
else
  echo "Unsupported shell"
fi
echo "RT_WS=$RT_WS"

echo "shell: $shell"
ROS2_WS="$RT_WS/../../ros2_ws"

MOVEIT2_SETUP="$ROS2_WS/moveit2_setup.sh"
if [[ -f "$MOVEIT2_SETUP" ]]; then
    . "$MOVEIT2_SETUP"
else
    echo "moveit2 set up failed"
    exit 1
fi

RT_SETUP="$RT_WS/install/local_setup.$shell"
if [[ -f "$RT_SETUP" ]]; then
    echo "rt_ws ($shell)"
    . "$RT_SETUP"

    if [[ ! -f "$HOME/.ros/fastdds.xml" ]]; then
        echo "Copying fastdds.xml to \"$HOME/.ros/\" ..."
        mkdir -p "$HOME/.ros"
        cp -v "$RT_WS/fastdds.xml" "$HOME/.ros/fastdds.xml"
    fi
fi

export WAYLAND_DISPLAY="wayland-0"
export DISPLAY=":0"
