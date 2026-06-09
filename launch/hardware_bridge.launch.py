import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_directory = get_package_share_directory('open_mower_next')
    default_yaml_path = os.path.join(
        share_directory, 'config', 'hardware', 'mainboard_serial_bridge.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'yaml_file',
            default_value=default_yaml_path,
            description='Full path to the mainboard serial bridge YAML file'
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value=os.environ.get('OM_MAINBOARD_SERIAL_PORT', '/dev/ttyAMA0'),
            description='Low-level mainboard serial device'
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value=os.environ.get('OM_MAINBOARD_BAUDRATE', '115200'),
            description='Low-level mainboard serial baudrate'
        ),
        Node(
            package='open_mower_next',
            executable='mainboard_serial_bridge',
            output='screen',
            parameters=[
                LaunchConfiguration('yaml_file'),
                {
                    'serial_port': LaunchConfiguration('serial_port'),
                    'baudrate': LaunchConfiguration('baudrate'),
                },
            ],
        ),
    ])
