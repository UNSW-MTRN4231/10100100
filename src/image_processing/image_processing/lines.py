import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformListener
from custom_messages.srv import PathClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import cv2
import numpy as np


from custom_messages.srv import PathClient
from tf2_ros.buffer import Buffer
from std_msgs.msg import Bool


class lines(Node):

    def __init__(self):
        super().__init__('lines')
        client_cb_group = MutuallyExclusiveCallbackGroup()
        sub_cb_group = MutuallyExclusiveCallbackGroup()
        transform_cb_group = MutuallyExclusiveCallbackGroup()
        # self.subscription = self.create_subscription(RobotAction, 'robot_action', self.topic_callback, 10)
        self.tf_broadcaster_ = TransformBroadcaster(self)  # Pass 'self' to the constructor
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)  # Pass 'self' to the constructor
        self.subscription = self.create_subscription(Bool, '/request', self.topic_callback, 10, callback_group=sub_cb_group)
        self.client = self.create_client(PathClient, 'path_service', callback_group=client_cb_group)

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = PathClient.Request()
        

    def handle_exceptions(self, source_frame, target_frame):
        try:
            corner1 = self.tf_listener.lookup_transform(target_frame, source_frame, rclpy.time.Time())
        except:
            self.get_logger().error(f"Error looking up transformation")

    def handle_service_request(self):
        self.req.colour = [1]
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info("recieved response")

        return self.future.result()
    
    # def find_homography_callback(self):


    def topic_callback(self, msg):

        response = self.handle_service_request()
        
        self.get_logger().info("recieved response")

        # assuming aruco markers go 1,2,3,4 clockwise. 1 is futhest away from robot base
        # paper is horezontal, simmilar to table.
        source_frame = "paper_corner_0"
        target_frame = "base_link"
        self.get_logger().info("Finding 1")
        # self.handle_exceptions(self, source_frame, target_frame)
        corner1 = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
        self.get_logger().info(str(corner1))

        source_frame = "paper_corner_1"
        target_frame = "base_link"
        self.get_logger().info("Finding 2")
        # self.handle_exceptions(self, source_frame, target_frame)
        corner2 = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
        self.get_logger().info(str(corner2))

        source_frame = "paper_corner_2"
        target_frame = "base_link"
        self.get_logger().info("Finding 3")
        # self.handle_exceptions(self, source_frame, target_frame)
        corner3 = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
        self.get_logger().info(str(corner3))

        source_frame = "paper_corner_3"    
        target_frame = "base_link"
        # self.handle_exceptions(self, source_frame, target_frame)
        self.get_logger().info("Finding 4")
        corner4 = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
        self.get_logger().info(str(corner4))

        # need to check if these produce messurments in the right sign. Could have gotten confuesed with axies orientations.
        paper_hight = corner1.transform.translation.x - corner3.transform.translation.x
        paper_lenght = corner1.transform.translation.x - corner2.transform.translation.x
        paper_ratio = paper_hight / paper_lenght

        # will add "crop" from camera length resolution "x" variable.
        # this will stop lines from warping during transfromation by keeping the hight/lenght ratio the same as the paper
        cam_height = response.height
        cam_length = response.width
        crop = paper_ratio*cam_length/cam_height
        cam_length = abs(cam_length + crop)
        


        src_points = np.array([
            [corner1.transform.translation.x, corner1.transform.translation.y],  # Point 1
            [corner2.transform.translation.x, corner2.transform.translation.y],  # Point 2
            [corner3.transform.translation.x, corner3.transform.translation.y],  # Point 3
            [corner4.transform.translation.x, corner4.transform.translation.y]   # Point 4
        ], dtype=np.float32)
        # Define the destination points (desired points)
        dst_points = np.array([
            [0, 0],  # New position for Point 1
            [cam_length, 0],  # New position for Point 2
            [cam_length, cam_height],  # New position for Point 3
            [0, cam_height]   # New position for Point 4
        ], dtype=np.float32)
        # Find the perspective transformation matrix (homography)
        H, _ = cv2.findHomography(src_points, dst_points)
        self.get_logger().info("Found Homodgraphy")
        # The matrix H now contains the transformation from src_points to dst_points

        x_path = []
        y_path = []
        z_path = []
        # take homography matrix and multiply it by the robot_action points to get global points
        for x, y in zip(response.x, response.y):
            transformed_point = np.dot(H, [x, y, 1])

            # Access the transformed coordinates
            transformed_x, transformed_y, w = transformed_point
            # # Normalize the coordinates (divide by w)
            transformed_x /= w
            transformed_y /= w

            # # Create a dynamic transformation from "base_frame" to "child_frame"
            # dynamic_transform = TransformStamped()
            # dynamic_transform.header.frame_id = 'base_frame'
            # dynamic_transform.child_frame_id = 'robot_action'
            # dynamic_transform.transform.translation.x = transformed_x  # Update translation
            # dynamic_transform.transform.translation.y = transformed_y
            # dynamic_transform.transform.translation.z = 10.0
            # dynamic_transform.transform.rotation.w = 0.0  # Update rotation

            # # Set the timestamp
            # dynamic_transform.header.stamp = self.get_clock().now().to_msg()
            # self.get_logger().info("Broadcasting....")
            # Broadcast the dynamic transformation
            self.get_logger().info("appending to path")
            x_path.append(transformed_x)
            y_path.append(transformed_y)
            z_path.append(w)
        
        self.get_logger().info(str(x_path))
            



def main(args=None):
    rclpy.init(args=args)
    node = lines()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
