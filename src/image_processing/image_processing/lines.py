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
from custom_messages.msg import RobotAction

from custom_messages.srv import PathClient
from tf2_ros.buffer import Buffer
from std_msgs.msg import Bool


class lines(Node):

    def __init__(self):
        super().__init__('lines')
        client_cb_group = MutuallyExclusiveCallbackGroup()
        sub_cb_group = MutuallyExclusiveCallbackGroup()
        pub_cb_group = MutuallyExclusiveCallbackGroup()
        # self.subscription = self.create_subscription(RobotAction, 'robot_action', self.topic_callback, 10)
        self.tf_broadcaster_ = TransformBroadcaster(self)  # Pass 'self' to the constructor
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)  # Pass 'self' to the constructor
        self.subscription = self.create_subscription(Bool, '/image_obtained', self.send_path_to_moveit, 10, callback_group=sub_cb_group)
        self.client = self.create_client(PathClient, 'path_service', callback_group=client_cb_group)
        self.publisher = self.create_publisher(RobotAction, '/robot_action', 10, callback_group=pub_cb_group)

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

        return self.future.result()
    
    # def find_homography_callback(self):

    def send_path_to_moveit(self, msg):

        if not msg.data: return
        response = self.handle_service_request()
        
        self.get_logger().info("recieved response")
        send_msg = RobotAction()
        send_msg.command = "draw"
        self.get_logger().info("packing x")
        send_msg.x = response.x
        self.get_logger().info("packing y")
        send_msg.y = response.y
        self.get_logger().info("about to send")
        self.publisher.publish(send_msg)
        self.get_logger().info("sending path")
        

            



def main(args=None):
    rclpy.init(args=args)
    node = lines()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
