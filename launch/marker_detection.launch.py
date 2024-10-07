from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare arguments
    declare_record_cmd = DeclareLaunchArgument(
        'record', default_value='0',
        description='Record video'
    )
    
    declare_marker_detector_cmd = DeclareLaunchArgument(
        'use_opencv', default_value='false',
        description='Use OpenCV-based marker detector node'
    )

    # Launch configurations
    record = LaunchConfiguration('record')
    use_opencv = LaunchConfiguration('use_opencv')

    # Conditional marker detector nodes
    marker_detector_node = Node(
        package='drone_auto_land',
        executable='marker_detector',
        name='marker_detector',
        condition=IfCondition(LaunchConfiguration('use_opencv').not_equals('true')),
    )

    marker_detector_opencv_node = Node(
        package='drone_auto_land',
        executable='marker_detector_open_cv.py',
        name='marker_detector_open_cv',
        condition=IfCondition(LaunchConfiguration('use_opencv').equals('true')),
    )

    frame_converter_node = Node(
        package='drone_auto_land',
        executable='frame_converter',
        name='frame_converter',
        parameters=[{'record': record}]
    )

    return LaunchDescription([
        declare_record_cmd,
        declare_marker_detector_cmd,
        marker_detector_node,
        marker_detector_opencv_node,
        frame_converter_node,
    ])
