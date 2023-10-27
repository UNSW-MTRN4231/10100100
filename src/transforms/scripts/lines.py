import rclpy
from tf2_py import Quaternion
from geometry_msgs.msg import TransformStamped, PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import String
import math
import threading
import time
from functools import partial
import string
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformListener

from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseArray, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from custom_messages import RobotAction


class lines(Node):

    def __init__(self):
    super().__init__('line')
    self.subscription = self.create_subscription(RobotAction, 'robot_action', self.topic_callback, 10 )
    tf_broadcaster_ = tf2_ros.TransformBroadcaster(node)
    tf_listener = TransformListener(node)


    def topic_callback(self, msg):

        # assuming aruco markers go 1,2,3,4 clockwise. 1 is futhest away from robot base
        # paper is horezontal, simmilar to table.
        source_frame = "paper_corner_1"
        target_frame = "base_frame"
        corner1 = tf_listener.lookup_transform(target_frame, source_frame, rclpy.time.Time())

        source_frame = "paper_corner_2"
        target_frame = "base_frame"
        corner2 = tf_listener.lookup_transform(target_frame, source_frame, rclpy.time.Time())

        source_frame = "paper_corner_3"
        target_frame = "base_frame"
        corner3 = tf_listener.lookup_transform(target_frame, source_frame, rclpy.time.Time())


        # need to check if these produce messurments in the right sign. Could have gotten confuesed with axies orientations.
        paper_hight = corner1.transform.translation.x - corner2.transform.translation.x
        paper_lenght = corner1.transform.translation.x - corner3.transform.translation.x
        paper_ratio = paper_hight / paper_lenght




        # index = iter(msg.x)
        # for i in index






def main(args=None):
    rclpy.init(args=args)
    node = BallDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()