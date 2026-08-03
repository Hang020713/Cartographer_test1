from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Bus manager
        Node(
            package='sensor_monitor',
            executable='rs485_bus_manager',
            name='rs485_bus_manager',
            parameters=[{'max_hold_time_ms': 1000}],
        ),
        
        # BMS node (no auto-polling)
        Node(
            package='sensor_monitor',
            executable='bms485_node',
            name='bms485_node',
            parameters=[{
                'serial_port': '/dev/ttyAMA3',
                'slave_id': 1,
                'resp_timeout_ms': 500,
            }],
        ),
        
        # Level transmitter (no auto-polling)
        Node(
            package='sensor_monitor',
            executable='level_transmitter_node',
            name='level_transmitter_node',
            parameters=[{
                'serial_port': '/dev/ttyAMA3',
                'slave_id': 2,
                'use_float_mode': True,
            }],
        ),
        
        # Coordinator (orchestrates polling sequence)
        Node(
            package='sensor_monitor',
            executable='rs485_coordinator',
            name='rs485_coordinator',
            parameters=[{
                'bms_poll_interval_ms': 2000,
                'level_poll_interval_ms': 2000,
                'bus_acquire_timeout_ms': 500,
            }],
        ),
    ])