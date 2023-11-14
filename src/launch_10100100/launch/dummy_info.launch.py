import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the 'dummy_camera_info' node
        Node(
            package="image_processing",
            executable="dummy_camera_info",
            name="dummy_camera_info",
            output="screen",
            # Add any node-specific parameters here
        ),

        # Launch the 'test_img_publisher' node
        Node(
            package="image_processing",
            executable="test_img_publisher",
            name="test_img_publisher",
            output="screen",
            # Add any node-specific parameters here
        ),
        # Launch the 'test_img_publisher' node
        # Node(
        #     package="image_processing",
        #     executable="dummy_smile_pub",
        #     name="dummy_smile_pub",
        #     output="screen",
        #     # Add any node-specific parameters here
        # ),
    ])