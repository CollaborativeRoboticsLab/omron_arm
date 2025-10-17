import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    tm_robot_ip = LaunchConfiguration('tm_robot_ip')
    tm_use_simulation = LaunchConfiguration('tm_use_simulation')

    declare_robot_ip = DeclareLaunchArgument(
            'tm_robot_ip',
            default_value='192.168.1.2',
            description='Target robot IP address'
        )

    declare_use_simulation = DeclareLaunchArgument(
            'tm_use_simulation',
            default_value='false',
            description='Use simulation mode (true/false)'
        )

    system_node = Node(
            package='tm_driver',
            executable='tm_driver',
            output='screen',
            parameters=[{
                'tm_robot_ip': tm_robot_ip,
                'tm_use_simulation': tm_use_simulation,
            }],
        )

    return LaunchDescription([
        declare_robot_ip,
        declare_use_simulation,
        system_node
    ])