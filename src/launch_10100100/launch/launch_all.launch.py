import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['./setupRealur5e.sh'],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['./camera.sh'],
            output='screen',
        ),
        # Launch the 'dummy_camera_info' node
        Node(
            package="image_processing",
            executable="contour_detection_node",
            name="contour_detection_node",
            output="screen",
            # Add any node-specific parameters here
        ),
        Node(
            package="image_processing",
            executable="smile_detector_node",
            name="smile_detector_node",
            output="screen",
            # Add any node-specific parameters here
        ),
        Node(
            package="image_processing",
            executable="lines",
            name="lines",
            output="screen",
            # Add any node-specific parameters here
        ),
        Node(
            package="transforms",
            executable="paper",
            name="paper",
            output="screen",
            # Add any node-specific parameters here
        ),
        Node(
            package="transforms",
            executable="static_camera_transform",
            name="static_camera_transform",
            output="screen",
            # Add any node-specific parameters here
        ),
        Node(
            package="util_arduino_serial",
            executable="util_arduino_serial",
            name="util_arduino_serial",
            output="screen",
            # Add any node-specific parameters here
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource('/home/mtrn/4231/10100100/src/path_planning/launch/move_command.launch.py'),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource('/home/mtrn/4231/10100100/src/ros2_aruco/launch/aruco_recognition.launch.py'),
        ),  
    ])


