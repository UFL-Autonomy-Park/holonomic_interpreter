from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    holonomic_interpreter_param_file = PathJoinSubstitution(
        [
            FindPackageShare("holonomic_interpreter"),
            "param",
            "holonomic_interpreter_param.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file", default_value=holonomic_interpreter_param_file
            ),
            Node(
                package="holonomic_interpreter",
                executable="holonomic_interpreter_node",
                name="holonomic_interpreter",
                parameters=[params_file],
                remappings=[
                    ("holo_cmd_vel", "cmd_vel_filtered"),
                    ("odom", "odometry/global"),
                    ("nonholo_cmd_vel", "cmd_vel"),
                ],
            ),
        ]
    )
