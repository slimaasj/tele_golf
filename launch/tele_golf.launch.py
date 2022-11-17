import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
import launch_ros

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tele_golf',
            namespace='Motor',
            executable='pyMotor',
            name='pyMotor'
        )
    ])
