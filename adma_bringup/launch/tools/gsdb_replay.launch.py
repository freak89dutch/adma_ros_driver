from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    gsdb_config_arg = DeclareLaunchArgument(
        'gsdb_config',
        default_value=PathJoinSubstitution(
            [FindPackageShare('adma_bringup'), 'config', 'tools', 'gsdb_replay_config.yaml']
        ),
    )
    gsdb_config = LaunchConfiguration('gsdb_config')
    # overwrite ROS arg here to prevent creating new gsdb file during replaying another one..
    log_gsdb_arg = DeclareLaunchArgument('log_gsdb', default_value='False')
    log_addon_delta_gsdb_arg = DeclareLaunchArgument('log_addon_delta', default_value='False')

    # set this to true if you want to replay GSDB and create a rosbag of it
    record_rosbag_arg = DeclareLaunchArgument('record_rosbag', default_value='True')

    log_level_arg = DeclareLaunchArgument('log_level', default_value='INFO')
    adma_namespace_arg = DeclareLaunchArgument('adma_namespace', default_value='genesys')

    adma_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('adma_bringup'),
                    'launch',
                    'adma_server.launch.py',
                ]
            )
        ),
        launch_arguments={'adma_server_config': gsdb_config}.items(),
    )

    adma_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('adma_bringup'),
                    'launch',
                    'admanet',
                    'admanet_driver.launch.py',
                ]
            )
        )
    )

    delta_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('adma_bringup'),
                    'launch',
                    'addondelta',
                    'addondelta_driver.launch.py',
                ]
            )
        ),
        launch_arguments={'addon_delta_config': gsdb_config}.items(),
    )

    gsdb_server = Node(
        package='adma_tools_cpp',
        executable='gsdb_server',
        output='screen',
        namespace=LaunchConfiguration('adma_namespace'),
        name='gsdb_server',
        parameters=[gsdb_config],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        on_exit=[
            LogInfo(msg=['GSDB replay done. Stopping everything...']),
            Shutdown(reason='launch is shutting down'),
        ],
    )

    return LaunchDescription(
        [
            # # args
            gsdb_config_arg,
            log_level_arg,
            adma_namespace_arg,
            log_gsdb_arg,
            record_rosbag_arg,
            log_addon_delta_gsdb_arg,
            # #  nodes
            adma_server,
            adma_driver,
            gsdb_server,
            delta_driver,
        ]
    )
