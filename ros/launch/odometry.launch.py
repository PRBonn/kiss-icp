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
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    current_pkg = FindPackageShare("kiss_icp")
    topic_arg = DeclareLaunchArgument(
        "topic", description="sensor_msg/PointCloud2 topic to process"
    )
    bagfile_arg = DeclareLaunchArgument("bagfile", default_value="")
    visualize_arg = DeclareLaunchArgument("visualize", default_value="true")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")

    kiss_icp_node = Node(
        package="kiss_icp",
        executable="kiss_icp_node",
        name="kiss_icp_node",
        output="screen",
        remappings=[("pointcloud_topic", LaunchConfiguration("topic"))],
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "publish_debug_clouds": LaunchConfiguration("visualize"),
            },
            PathJoinSubstitution([current_pkg, "config", "kiss_icp.yaml"]),
        ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution([current_pkg, "rviz", "kiss_icp.rviz"]),
        ],
        condition=IfCondition(LaunchConfiguration("visualize")),
    )
    bagfile_play = ExecuteProcess(
        cmd=["ros2", "bag", "play", LaunchConfiguration("bagfile")],
        output="screen",
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration("bagfile"), "' != ''"])),
    )
    return LaunchDescription(
        [
            topic_arg,
            bagfile_arg,
            visualize_arg,
            use_sim_time_arg,
            kiss_icp_node,
            rviz_node,
            bagfile_play,
        ]
    )
