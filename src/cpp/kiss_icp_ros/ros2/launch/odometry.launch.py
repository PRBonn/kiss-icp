from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()
    current_pkg = FindPackageShare('kiss_icp')

    # Declare the launch arguments
    declare_topic = DeclareLaunchArgument(
        'topic', default_value='points', description='Pointcloud topic name'
    )
    ld.add_action(declare_topic)

    declare_visualize = DeclareLaunchArgument(
        'visualize', default_value='true', description='Whether to launch RViz'
    )
    ld.add_action(declare_visualize)

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [current_pkg, 'params', 'default.yaml']
        ),
        description='Full path to the ROS2 parameters file to use',
    )
    ld.add_action(declare_params_file)

    # Add the nodes
    odometry_node = Node(
        package='kiss_icp',
        executable='odometry_node',
        name='odometry_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[('pointcloud_topic', LaunchConfiguration('topic'))],
    )
    ld.add_action(odometry_node)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output={'both': 'log'},
        arguments=['-d', PathJoinSubstitution([current_pkg, 'rviz', 'kiss_icp.rviz'])],
        condition=IfCondition(LaunchConfiguration('visualize'))
    )
    ld.add_action(rviz_node)

    return ld
