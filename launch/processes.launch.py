from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare an argument
    declare_headless_cmd = DeclareLaunchArgument(
        'headless', default_value='0',
        description='Run in headless mode')
    
    declare_simulation_cmd = DeclareLaunchArgument( 
        'simulation', default_value='1',
        description='Run in simulation mode')
    
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
        parameters=[{'headless': LaunchConfiguration('headless')},
                    {'simulation': LaunchConfiguration('simulation')}]
    )

    return LaunchDescription([
        declare_headless_cmd,
        camera_bridge_node,
        declare_simulation_cmd,
        processes_node
    ])