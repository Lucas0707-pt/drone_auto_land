from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare an argument

    processes_node = Node(
        package='drone_auto_land',
        executable='processes',
        name='processes',
        output='screen',
    )

    return LaunchDescription([
        processes_node
    ])