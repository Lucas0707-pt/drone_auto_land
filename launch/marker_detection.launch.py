from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import LaunchConfigurationEquals

def generate_launch_description():
    # Declare arguments
    declare_record_cmd = DeclareLaunchArgument(
        'record', default_value='0',
        description='Record video'
    )
    
    declare_marker_detector_cmd = DeclareLaunchArgument(
        'use_opencv', default_value='0',
        description='Use OpenCV-based marker detector node'
    )

    # Launch configurations
    record = LaunchConfiguration('record')
    use_opencv = LaunchConfiguration('use_opencv')

    # Log the values of the configurations
    log_use_opencv = LogInfo(msg=['use_opencv: ', use_opencv])

    # Conditional marker detector nodes
    marker_detector_node = Node(
        package='drone_auto_land',
        executable='marker_detector',
        name='marker_detector',
        condition=LaunchConfigurationEquals('use_opencv', '0')
    )

    marker_detector_opencv_node = Node(
        package='drone_auto_land',
        executable='marker_detector_open_cv',
        name='marker_detector_open_cv',
        condition=LaunchConfigurationEquals('use_opencv', '1')
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
        log_use_opencv,
        marker_detector_node,
        marker_detector_opencv_node,
        frame_converter_node,
    ])
