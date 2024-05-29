from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():

    declare_log_cmd = DeclareLaunchArgument(
        'log', default_value='0',
        description='Enable or disable logging'
    )

    log = LaunchConfiguration('log')

    control_node = Node(
        package='drone_auto_land',
        executable='control_node',
        name='control_node',
    )

    data_logger_node = Node(
        package='drone_auto_land',
        executable='data_logger',
        name='data_logger',
        condition=IfCondition(log),
    )

    return LaunchDescription([
        declare_log_cmd,
        control_node,
        data_logger_node
    ])