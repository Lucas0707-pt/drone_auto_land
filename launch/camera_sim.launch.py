from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_auto_land',
            executable='uav_camera_sim',
            name='uav_camera_sim',
            output='screen'
        ),
    ])
