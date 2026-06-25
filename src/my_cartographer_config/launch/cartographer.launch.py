import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_cartographer_config')
    configuration_directory = os.path.join(pkg_dir, 'config')
    configuration_basename = 'my_robot.lua'

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=[
                '-configuration_directory', configuration_directory,
                '-configuration_basename', configuration_basename,
            ],
            remappings=[
                ('scan', '/scan'),
                ('imu', '/imu/data')
            ]
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=['-resolution', '0.1',
                '-publish_period_src', '1.0']
        ),
    ])
