from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    control_node = Node(
        package='drone_auto_land',
        executable='control_node',
        name='control_node',
    )

    data_logger_node = Node(
        package='drone_auto_land',
        executable='data_logger',
        name='data_logger',
    )

    return LaunchDescription([
        control_node,
        data_logger_node
    ])