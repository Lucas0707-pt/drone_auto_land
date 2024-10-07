from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare an argument

    camera_bridge_node = Node(
        package='drone_auto_land',
        executable='camera_bridge',
        name='camera_bridge',
    )

    processes_node = Node(
        package='drone_auto_land',
        executable='processes',
        name='processes',
        output='screen',
    )

    return LaunchDescription([
        camera_bridge_node,
        processes_node
    ])