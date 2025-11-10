import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share_dir = get_package_share_directory('adma_bringup')

    rviz_path = os.path.join(pkg_share_dir, 'rviz', 'display.rviz')
    urdf_path = os.path.join(pkg_share_dir, 'urdf', 'auto.urdf.xacro')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': ParameterValue(
                    Command(['xacro ', str(urdf_path)]), value_type=str
                ),
                'use_sim_time': True,
            }
        ],
    )

    start_rviz_arg = DeclareLaunchArgument('start_rviz', default_value='False')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_path],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('start_rviz')),
    )

    return LaunchDescription(
        [
            start_rviz_arg,
            robot_state_publisher_node,
            rviz_node,
            # map_start,
            # odom_node,
        ]
    )
