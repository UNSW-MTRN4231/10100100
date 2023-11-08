import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch the 'dummy_camera_info' node
        Node(
            package="image_processing",
            executable="contour_detection_node",
            name="contour_detection_node",
            output="screen",
            # Add any node-specific parameters here
        ),

        # Launch the 'test_img_publisher' node
        Node(
            package="image_processing",
            executable="lines",
            name="lines",
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource("move_command.launch.py.py"),
            launch_arguments={"enable_info": LaunchConfiguration("enable_info")}
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource("aruco_recognition.launch.py.py"),
            launch_arguments={"enable_info": LaunchConfiguration("enable_info")}
        ),
        Node(
            package="image_processing",
            executable="lines",
            name="lines",
            output="screen",
            # Add any node-specific parameters here
        ),
    ])