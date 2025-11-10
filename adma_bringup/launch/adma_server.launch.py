from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def generate_launch_description():
    # first load yaml file with parameters
    adma_server_config_arg = DeclareLaunchArgument(
        'adma_server_config',
        default_value=PathJoinSubstitution(
            [FindPackageShare('adma_bringup'), 'config', 'adma_server_config.yaml']
        ),
    )
    adma_server_config = LaunchConfiguration('adma_server_config')

    log_level_arg = DeclareLaunchArgument('log_level', default_value='INFO')
    adma_namespace_arg = DeclareLaunchArgument('adma_namespace', default_value='genesys')

    def yaml_handling(context, *args, **kwargs):
        # extract params to allow optional injecting them by CLI
        with open(adma_server_config.perform(context), 'r') as f:
            server_params = yaml.safe_load(f)['/**']['adma_server']['ros__parameters']

        adma_ip_arg = DeclareLaunchArgument(
            'adma_ip',
            default_value=server_params.pop('adma_ip', '192.168.88.255'),
            description='IP adress of the ADMA to connect to',
        )
        admanet_port_arg = DeclareLaunchArgument(
            'admanet_port',
            default_value=str(server_params.pop('admanet_port', 11021)),
            description='UDP port for the ADMAnet datastream',
        )
        addondelta_port_arg = DeclareLaunchArgument(
            'addondelta_port',
            default_value=str(server_params.pop('addondelta_port', 1025)),
            description='UDP port for the AddonDelta datastream',
        )
        time_mode_arg = DeclareLaunchArgument(
            'time_mode',
            default_value=str(server_params.pop('time_mode', 0)),
            description='time source (0 = INS time / 1 = current ROS time)',
        )

        adma_server_params = {
            'adma_ip': LaunchConfiguration('adma_ip'),
            'admanet_port': LaunchConfiguration('admanet_port'),
            'addondelta_port': LaunchConfiguration('addondelta_port'),
            'time_mode': LaunchConfiguration('time_mode'),
        }

        return [
            # args
            adma_ip_arg,
            admanet_port_arg,
            addondelta_port_arg,
            time_mode_arg,
            # nodes
            Node(
                package='adma_core_lib',
                executable='adma_server',
                output='screen',
                namespace=LaunchConfiguration('adma_namespace'),
                name='adma_server',
                parameters=[adma_server_params],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            ),
        ]

    return LaunchDescription(
        [
            # # args
            adma_server_config_arg,
            log_level_arg,
            adma_namespace_arg,
            OpaqueFunction(function=yaml_handling),
        ]
    )
