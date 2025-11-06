from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([

        # Allows overriding the params file from command line
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("auto_marker_docking"),
                "config",
                "params.yaml"
            ]),
            description="Full path to the parameters file."
        ),

        # Auto docking node
        Node(
            package="auto_marker_docking",
            executable="auto_marker_docking_node",
            name="auto_marker_docking_node",
            output="screen",
            parameters=[params_file]
        ),

        # Simple debug viewer node (optional)
        Node(
            package="auto_marker_docking",
            executable="aruco_simple_detector",
            name="aruco_simple_detector",
            output="screen"
        ),
    ])
