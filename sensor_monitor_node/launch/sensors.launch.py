from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_monitor',
            executable='sht3x_node',
            name='sht3x_node',
            output='screen',
        ),
        Node(
            package='sensor_monitor',
            executable='ina4230_node',
            name='ina4230_node',
            output='screen',
        ),
        Node(
            package='sensor_monitor',
            executable='level_transmitter_node',
            name='level_transmitter_node',
            output='screen',
        ),
        Node(
            package='sensor_monitor',
            executable='bms485_node',
            name='bms485_node',
            output='screen',
        ),
    ])
