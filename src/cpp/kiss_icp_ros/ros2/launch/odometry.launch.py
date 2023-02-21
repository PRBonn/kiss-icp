from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    current_pkg = FindPackageShare("kiss_icp")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "topic",
                default_value="points",
                description="Pointcloud topic name",
            ),
            DeclareLaunchArgument(
                "visualize",
                default_value="true",
                description="Whether to launch RViz",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution([current_pkg, "params", "default.yaml"]),
                description="Full path to the ROS2 parameters file to use",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output={"both": "log"},
                arguments=["-d", PathJoinSubstitution([current_pkg, "rviz", "kiss_icp.rviz"])],
                condition=IfCondition(LaunchConfiguration("visualize")),
            ),
            Node(
                package="kiss_icp",
                executable="odometry_node",
                name="odometry_node",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
                remappings=[("pointcloud_topic", LaunchConfiguration("topic"))],
            ),
        ]
    )
