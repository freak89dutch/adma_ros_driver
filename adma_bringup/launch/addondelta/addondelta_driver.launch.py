from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # read default params
    addon_delta_config_arg = DeclareLaunchArgument(
        'addon_delta_config',
        default_value=PathJoinSubstitution(
            [
                FindPackageShare('adma_bringup'),
                'config',
                'addondelta',
                'addon_delta_driver_config.yaml',
            ]
        ),
    )
    addon_delta_config = LaunchConfiguration('addon_delta_config')
    # read path for dynamic channel mapping
    addon_delta_mapping_path_arg = DeclareLaunchArgument(
        'addon_delta_mapping_path',
        default_value=PathJoinSubstitution(
            [
                FindPackageShare('adma_bringup'),
                'config',
                'addondelta',
                'protocols',
            ]
        ),
    )
    addon_delta_mapping_path = LaunchConfiguration('addon_delta_mapping_path')

    # further default params
    log_level_arg = DeclareLaunchArgument('log_level', default_value='INFO')
    adma_namespace_arg = DeclareLaunchArgument('adma_namespace', default_value='genesys')

    adma_delta_driver = Node(
        package='adma_ros2_delta_driver',
        executable='adma_delta_driver',
        output='screen',
        namespace=LaunchConfiguration('adma_namespace'),
        name='adma_ros2_delta_driver',
        parameters=[addon_delta_config, {'addon_delta_mapping_path': addon_delta_mapping_path}],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    return LaunchDescription(
        [
            # # args
            addon_delta_config_arg,
            addon_delta_mapping_path_arg,
            log_level_arg,
            adma_namespace_arg,
            # #  nodes
            adma_delta_driver,
        ]
    )
