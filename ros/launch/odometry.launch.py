# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# This configuration parameters are not exposed thorught the launch system, meaning you can't modify
# those throw the ros launch CLI. If you need to change these values, you could write your own
# launch file and modify the 'parameters=' block from the Node class.
class config:
    # Preprocessing
    max_range: float = 100.0
    min_range: float = 0.0
    deskew: bool = True

    #  Mapping parameters
    voxel_size: float = max_range / 100.0
    max_points_per_voxel: int = 20

    # Adaptive threshold
    initial_threshold: float = 2.0
    min_motion_th: float = 0.1

    # Registration
    max_num_iterations: int = 500  #
    convergence_criterion: float = 0.0001
    max_num_threads: int = 0

    # Covariance diagonal values
    position_covariance: float = 0.1
    orientation_covariance: float = 0.1


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # ROS configuration
    pointcloud_topic = LaunchConfiguration("topic")
    visualize = LaunchConfiguration("visualize", default="true")

    # Optional ros bag play
    bagfile = LaunchConfiguration("bagfile", default="")

    # tf tree configuration, these are the likely parameters to change and nothing else
    base_frame = LaunchConfiguration("base_frame", default="")  # (base_link/base_footprint)
    lidar_odom_frame = LaunchConfiguration("lidar_odom_frame", default="odom_lidar")
    publish_odom_tf = LaunchConfiguration("publish_odom_tf", default=True)
    invert_odom_tf = LaunchConfiguration("invert_odom_tf", default=True)

    # KISS-ICP node
    kiss_icp_node = Node(
        package="kiss_icp",
        executable="kiss_icp_node",
        name="kiss_icp_node",
        output="screen",
        remappings=[
            ("pointcloud_topic", pointcloud_topic),
        ],
        parameters=[
            {
                # ROS node configuration
                "base_frame": base_frame,
                "lidar_odom_frame": lidar_odom_frame,
                "publish_odom_tf": publish_odom_tf,
                "invert_odom_tf": invert_odom_tf,
                # KISS-ICP configuration
                "max_range": config.max_range,
                "min_range": config.min_range,
                "deskew": config.deskew,
                "max_points_per_voxel": config.max_points_per_voxel,
                "voxel_size": config.voxel_size,
                # Adaptive threshold
                "initial_threshold": config.initial_threshold,
                "min_motion_th": config.min_motion_th,
                # Registration
                "max_num_iterations": config.max_num_iterations,
                "convergence_criterion": config.convergence_criterion,
                "max_num_threads": config.max_num_threads,
                # Fixed covariances
                "position_covariance": config.position_covariance,
                "orientation_covariance": config.orientation_covariance,
                # ROS CLI arguments
                "publish_debug_clouds": visualize,
                "use_sim_time": use_sim_time,
            },
        ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution([FindPackageShare("kiss_icp"), "rviz", "kiss_icp.rviz"]),
        ],
        condition=IfCondition(visualize),
    )

    bagfile_play = ExecuteProcess(
        cmd=["ros2", "bag", "play", "--rate", "1", bagfile, "--clock", "1000.0"],
        output="screen",
        condition=IfCondition(PythonExpression(["'", bagfile, "' != ''"])),
    )

    return LaunchDescription(
        [
            kiss_icp_node,
            rviz_node,
            bagfile_play,
        ]
    )
