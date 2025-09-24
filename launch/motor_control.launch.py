#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'publish_rate_us',
            default_value='2000',
            description='Publishing rate in microseconds'
        ),
        
        Node(
            package='motor_control_sdk',
            executable='motor_driver_node',
            name='motor_driver',
            output='screen',
            parameters=[{
                'publish_rate_us': LaunchConfiguration('publish_rate_us')
            }]
        )
    ])
