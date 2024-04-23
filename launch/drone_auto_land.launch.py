from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    declare_record_cmd = DeclareLaunchArgument(
        'record', default_value='0',
        description='Record video')
    
    marker_detector_node = Node(
        package='drone_auto_land',
        executable='marker_detector',
        name='marker_detector',
    )

    frame_converter_node = Node(
        package='drone_auto_land',
        executable='frame_converter',
        name='frame_converter',
        parameters=[{'record': LaunchConfiguration('record')}]
    )

    control_node = Node(
        package='drone_auto_land',
        executable='control_node',
        name='control_node',
    )

    return LaunchDescription([
        declare_record_cmd,
        declare_record_cmd,
        marker_detector_node,
        frame_converter_node,
        #control_node
    ])
