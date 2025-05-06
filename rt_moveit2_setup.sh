if [ -n "$BASH_VERSION" ]; then
    export RT_WS="$(cd -- "$(dirname "${BASH_SOURCE[0]}")" > /dev/null 2>&1; pwd -P)"
    SHELL="bash"
elif [ -n "$ZSH_VERSION" ]; then
    export RT_WS="$(cd -- "$(dirname "${(%):-%x}")" > /dev/null 2>&1; pwd -P)"
    SHELL="zsh"
else
  echo "Unsupported shell"
fi
echo "RT_WS: $RT_WS"

echo "shell: $SHELL"
ROS2_WS="$RT_WS/../../ros2_ws"

MOVEIT2_SETUP="$ROS2_WS/moveit2_setup.sh"
if [[ -f "$MOVEIT2_SETUP" ]]; then
    . "$MOVEIT2_SETUP"
else
    echo "moveit2 set up failed"
    exit 1
fi

RT_MOVEIT_SETUP="$RT_WS/install/local_setup.$SHELL"
if [[ -f "$RT_MOVEIT_SETUP" ]]; then
    echo "rt_moveit2 ($SHELL)"
    . "$RT_MOVEIT_SETUP"

    if [[ ! -f "$HOME/.ros/fastdds.xml" ]]; then
        echo "Copying fastdds.xml to \"$HOME/.ros/\" ..."
        mkdir -p "$HOME/.ros"
        cp -v "$RT_WS/fastdds.xml" "$HOME/.ros/fastdds.xml"
    fi
fi
