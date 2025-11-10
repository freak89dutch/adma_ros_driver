from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
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
    )
    admanet_mapping_path = LaunchConfiguration('admanet_mapping_path')

    bag_record_config = PathJoinSubstitution(
        [FindPackageShare('adma_ros2_driver'), 'config', 'record_bag_qos_profile.yaml']
    )

    bag_record_config_arg = DeclareLaunchArgument(
        'bag_record_config', default_value=bag_record_config
    )

    # further default params
    log_level_arg = DeclareLaunchArgument('log_level', default_value='INFO')
    adma_namespace_arg = DeclareLaunchArgument('adma_namespace', default_value='genesys')

    rosbag_file_arg = DeclareLaunchArgument('rosbag_path', default_value='./')
    # parameter for GSDB logging, used for ADMA-PP ####
    log_gsdb_arg = DeclareLaunchArgument('log_gsdb', default_value='False')
    log_addon_delta_gsdb_arg = DeclareLaunchArgument('log_addon_delta', default_value='False')
    raw_data_topic = 'adma/data_raw'

    # parameters for recording data into a rosbag ###
    record_ros_bag_arg = DeclareLaunchArgument('record_rosbag', default_value='False')
    # list of desired topic to record. just comment/uncomment the entries you need
    recorded_topics = [
        # '/genesys/adma/data_raw', # unnecessary since its redundant logged in gsdb
        # '/genesys/adma/data', # v3.3.3
        '/genesys/adma/data_scaled',  # v3.3.5
        '/genesys/adma/status',
        '/genesys/adma/fix',
        '/genesys/adma/imu',
        '/genesys/adma/heading',
        '/genesys/adma/velocity',
    ]

    adma_driver = Node(
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
            ('adma/data_raw', 'adma/admanet_raw'),
            ('adma/data_scaled', 'adma/data_scaled'),
            ('adma/status', 'adma/status'),
            ('adma/fix', 'adma/fix'),
            ('adma/imu', 'adma/imu'),
            ('adma/heading', 'adma/heading'),
            ('adma/velocity', 'adma/velocity'),
        ],
    )

    rosbag_recorder = ExecuteProcess(
        # cmd=['ros2', 'bag', 'record', '-s', 'mcap', '--all', '--use-sim-time'],
        cmd=[
            'ros2',
            'bag',
            'record',
            '--all',
            '--use-sim-time',
            '--max-cache-size',
            '4096',
            '--qos-profile-overrides-path',
            LaunchConfiguration('bag_record_config'),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('record_rosbag')),
    )

    gsdb_logger = Node(
        package='adma_tools_cpp',
        executable='bag2gsdb_converter',
        output='screen',
        namespace=LaunchConfiguration('adma_namespace'),
        name='bag2gsdb',
        parameters=[
            {
                'rosbag_path': LaunchConfiguration('rosbag_path'),
                'log_addon_delta': LaunchConfiguration('log_addon_delta'),
            }
        ],
        remappings=[('adma/data_raw', raw_data_topic)],
        condition=IfCondition(LaunchConfiguration('log_gsdb')),
    )

    return LaunchDescription(
        [
            # # args
            admanet_driver_config_arg,
            admanet_mapping_path_arg,
            bag_record_config_arg,
            log_level_arg,
            record_ros_bag_arg,
            rosbag_file_arg,
            log_gsdb_arg,
            adma_namespace_arg,
            log_addon_delta_gsdb_arg,
            # #  nodes
            adma_driver,
            # rosbag_recorder,
            # gsdb_logger,
        ]
    )
