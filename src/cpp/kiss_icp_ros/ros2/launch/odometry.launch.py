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
            # ROS2 parameters
            DeclareLaunchArgument("bagfile", default_value=""),
            DeclareLaunchArgument("visualize", default_value="true"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("child_frame", default_value="base_link"),
            DeclareLaunchArgument("topic", default_value=""),
            # KISS-ICP parameters
            DeclareLaunchArgument("deskew", default_value="false"),
            DeclareLaunchArgument("max_range", default_value="100.0"),
            DeclareLaunchArgument("min_range", default_value="5.0"),
            DeclareLaunchArgument("voxel_size", default_value=""),
            Node(
                package="kiss_icp",
                executable="odometry_node",
                name="odometry_node",
                output="screen",
                remappings=[("pointcloud_topic", LaunchConfiguration("topic"))],
                parameters=[
                    {
                        "odom_frame": LaunchConfiguration("odom_frame"),
                        "child_frame": LaunchConfiguration("child_frame"),
                        "max_range": LaunchConfiguration("max_range"),
                        "min_range": LaunchConfiguration("min_range"),
                        "deskew": LaunchConfiguration("deskew"),
                        "voxel_size": LaunchConfiguration("voxel_size"),
                        "frame_rate": 10,
                        "max_points_per_voxel": 20,
                        "initial_threshold": 2.0,
                        "min_motion_th": 0.1,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output={"both": "log"},
                arguments=["-d", PathJoinSubstitution([current_pkg, "rviz", "kiss_icp.rviz"])],
                condition=IfCondition(LaunchConfiguration("visualize")),
            ),
        ]
    )
