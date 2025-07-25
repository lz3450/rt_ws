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

ROS2_SETUP="$ROS2_WS/ros2_humble/install/local_setup.$shell"
if [[ -f "$ROS2_SETUP" ]]; then
    echo "ros2 humble ($shell)"
    . "$ROS2_SETUP"
else
    echo "failed to set up ros2 humble"
    exit 1
fi

MOVEIT2_SETUP="$ROS2_WS/moveit2_humble/install/local_setup.$shell"
if [[ -f "$MOVEIT2_SETUP" ]]; then
    echo "moveit2 ($shell)"
    . "$MOVEIT2_SETUP"
else
    echo "failed to set up moveit2"
    exit 1
fi

NAV2_SETUP="$ROS2_WS/nav2_humble/install/local_setup.$shell"
if [[ -f "$NAV2_SETUP" ]]; then
    echo "nav2 ($shell)"
    . "$NAV2_SETUP"
else
    echo "failed to set up nav2"
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
