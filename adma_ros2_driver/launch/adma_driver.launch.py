from launch import LaunchDescription
from launch.substitutions import TextSubstitution, PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

        adma_ip_address = TextSubstitution(text="0.0.0.0")
        adma_port = TextSubstitution(text="1040")

        driver_config = PathJoinSubstitution([FindPackageShare('adma_ros2_driver'), 'config', 'driver_config.yaml'])

        adma_ip_arg = DeclareLaunchArgument('destination_ip', default_value=adma_ip_address, description='IP address of the ADMA system')
        adma_port_arg = DeclareLaunchArgument('destination_port', default_value=adma_port, description='Broadcast port of the ADMA system')
        performance_check_arg = DeclareLaunchArgument('use_performance_check', default_value='True', description='True if timing check is required')
        driver_config_arg = DeclareLaunchArgument('driver_config', default_value=driver_config)
        log_level_arg = DeclareLaunchArgument('log_level', default_value='WARN')

        adma_driver = Node(
                package='adma_ros2_driver',
                executable='adma_driver',
                output='screen',
                namespace='genesys',
                name='adma_ros2_driver',
                parameters=[LaunchConfiguration('driver_config')],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        )
        
        return LaunchDescription([
                # # args
                adma_ip_arg,
                adma_port_arg,
                performance_check_arg,
                driver_config_arg,
                log_level_arg,
                # #  nodes
                adma_driver
        ])