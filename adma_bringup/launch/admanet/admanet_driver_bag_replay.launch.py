from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # read default params
    admanet_driver_config_arg = DeclareLaunchArgument(
        'admanet_driver_config',
        default_value=PathJoinSubstitution(
            [FindPackageShare('adma_bringup'), 'config', 'admanet', 'admanet_driver_config.yaml']
        ),
        description='Path of the YAML file which contains the ROS parameters of this setup',
    )
    admanet_driver_config = LaunchConfiguration('admanet_driver_config')

    # read path for dynamic channel mapping
    admanet_mapping_path_arg = DeclareLaunchArgument(
        'admanet_mapping_path',
        default_value=PathJoinSubstitution(
            [
                FindPackageShare('adma_bringup'),
                'config',
                'admanet',
                'protocols',
            ]
        ),
        description='folder of the JSON file that is used for mapping the UDP packets',
    )
    admanet_mapping_path = LaunchConfiguration('admanet_mapping_path')

    # further default params
    log_level_arg = DeclareLaunchArgument('log_level', default_value='INFO')
    adma_namespace_arg = DeclareLaunchArgument('adma_namespace', default_value='genesys')

    # set your path to the recorded data (db3/mcap)
    rosbag_file_arg = DeclareLaunchArgument(
        'rosbag_path',
        default_value='./record.db3',
        description='absolute path with filename of your recorded data to replay',
    )
    rosbag_file = LaunchConfiguration('rosbag_path')
    # optional increase the replay rate
    rosbag_replay_rate_arg = DeclareLaunchArgument(
        'rosbag_rate', default_value='1', description='rate to replay the rosbag'
    )
    rosbag_replay_rate = LaunchConfiguration('rosbag_rate')

    # use this if you use "old" records before it was renamed to admanet_raw
    raw_data_topic = 'adma/data_raw'

    admanet_driver = Node(
        package='adma_ros2_driver',
        executable='adma_driver',
        output='screen',
        namespace=LaunchConfiguration('adma_namespace'),
        name='adma_ros2_driver',
        parameters=[admanet_driver_config, {'admanet_mapping_path': admanet_mapping_path}],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=[
            # left=from / right=to (so publish the origin left on the new right topic)
            ('adma/data', 'adma/data'),
            ('adma/admanet_raw', raw_data_topic),
            ('adma/data_scaled', 'adma/data_scaled'),
            ('adma/status', 'adma/status'),
            ('adma/fix', 'adma/fix'),
            ('adma/imu', 'adma/imu'),
            ('adma/heading', 'adma/heading'),
            ('adma/velocity', 'adma/velocity'),
        ],
    )

    # Play the rosbag and register event handler to kill all nodes, once the rosbag is finished
    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', rosbag_file, '--rate', rosbag_replay_rate],
    )
    rosbag_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=rosbag_play,
            on_exit=[
                EmitEvent(event=Shutdown(reason='rosbag finished')),
                LogInfo(msg='rosbag finished, shutting down'),
            ],
        )
    )

    return LaunchDescription(
        [
            # # args
            admanet_driver_config_arg,
            admanet_mapping_path_arg,
            log_level_arg,
            rosbag_file_arg,
            rosbag_replay_rate_arg,
            adma_namespace_arg,
            # #  nodes
            admanet_driver,
            rosbag_play,
            rosbag_exit_handler,
        ]
    )
