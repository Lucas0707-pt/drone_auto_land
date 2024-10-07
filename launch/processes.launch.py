from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare an argument
    declare_headless_cmd = DeclareLaunchArgument(
        'headless', default_value='0',
        description='Run in headless mode')

    # Declare an argument
    processes_node = Node(
        package='drone_auto_land',
        executable='processes',
        name='processes',
        output='screen',
    )

    return LaunchDescription([
        declare_headless_cmd,
        processes_node
    ])