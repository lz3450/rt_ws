from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace

from nav2_common.launch import RewrittenYaml


ARGUMENTS = [
    DeclareLaunchArgument("use_sim_time", default_value="false", choices=["true", "false"], description="Use sim time"),
    DeclareLaunchArgument("sync", default_value="true", choices=["true", "false"], description="Use synchronous SLAM"),
    DeclareLaunchArgument("namespace", default_value="", description="Robot namespace"),
]


def generate_launch_description():
    pkg_rt_tb4_navigation = get_package_share_directory("rt_tb4_navigation")

    sync = LaunchConfiguration("sync")
    namespace = LaunchConfiguration("namespace")

    slam_params_arg = DeclareLaunchArgument(
        "params",
        default_value=PathJoinSubstitution([pkg_rt_tb4_navigation, "config", "slam.yaml"]),
        description="Robot namespace",
    )

    slam_params = RewrittenYaml(
        source_file=LaunchConfiguration("params"), root_key=namespace, param_rewrites={}, convert_types=True
    )

    remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
        ("/scan", "scan"),
        ("/map", "map"),
        ("/map_metadata", "map_metadata"),
    ]

    slam = GroupAction(
        [
            PushRosNamespace(namespace),
            Node(
                package="slam_toolbox",
                executable="sync_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[slam_params, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
                remappings=remappings,
                condition=IfCondition(sync),
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[slam_params, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
                remappings=remappings,
                condition=UnlessCondition(sync),
            ),
        ]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(slam_params_arg)
    ld.add_action(slam)
    return ld
