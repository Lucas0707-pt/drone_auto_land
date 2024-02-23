from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        #processes.py node
        Node(
            package='drone_auto_land',
            executable='processes',
            name='processes',
            output='screen'
        ),
    ])

