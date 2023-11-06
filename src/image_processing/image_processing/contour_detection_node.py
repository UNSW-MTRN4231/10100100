import numpy as np
from custom_messages.srv import PathClient
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformListener
from tf2_ros.buffer import Buffer
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import cv2 as cv

class ContourDetectionNode(Node):

    def __init__(self):
        super().__init__('contour_detection_node')
        client_cb_group = MutuallyExclusiveCallbackGroup()
        sub_cb_group = MutuallyExclusiveCallbackGroup()
        transform_cb_group = MutuallyExclusiveCallbackGroup()
        
        self.subscription = self.create_subscription(
            Image,
            'person_snapshot',  # replace the former node name
            self.image_callback,
            10, callback_group=sub_cb_group)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer_ = self.create_timer(1.0, self.find_homography, callback_group=transform_cb_group)
        self.srv = self.create_service(PathClient, 'path_service', self.callback, callback_group=client_cb_group)

    def find_homography(self):
        # assuming aruco markers go 1,2,3,4 clockwise. 1 is futhest away from robot base
        # paper is horezontal, simmilar to table.
        source_frame = "paper_corner_0"
        target_frame = "base_link"
        try:
            corner1 = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
        except:
            self.get_logger().info("Can't find canvas")
            return

        source_frame = "paper_corner_1"
        target_frame = "base_link"
        try:
            corner2 = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
        except:
            self.get_logger().info("Can't find canvas")
            return

        source_frame = "paper_corner_2"
        target_frame = "base_link"
        try:
            corner3 = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
        except:
            self.get_logger().info("Can't find canvas")
            return

        source_frame = "paper_corner_3"    
        target_frame = "base_link"
        try:
            corner4 = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
        except:
            self.get_logger().info("Can't find canvas")
            return


        # need to check if these produce messurments in the right sign. Could have gotten confuesed with axies orientations.
        paper_hight = corner1.transform.translation.x - corner3.transform.translation.x
        paper_lenght = corner1.transform.translation.x - corner2.transform.translation.x
        paper_ratio = paper_hight / paper_lenght

        # will add "crop" from camera length resolution "x" variable.
        # this will stop lines from warping during transfromation by keeping the hight/lenght ratio the same as the paper
        cam_height = 480
        cam_length = 640
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
        self.H, _ = cv.findHomography(src_points, dst_points)

    def callback(self, request, response):
        self.get_logger().info("Recieved Request")
        # Process the request containing an array of two integers
        index = request.colour[0]
        # Initialize empty lists for x and y values
        x = []
        y = []
        z = []
        # Open the text file for reading
        with open("test.txt", "r") as file:
            # Read each line in the file
            for line in file:
                # Split each line into two values using a space as the delimiter
                values = line.split()
                if len(values) == 2:
                    x_value, y_value = map(float, values)
                    transformed_point = np.dot(self.H, [x_value, y_value, 1])

                    # Access the transformed coordinates
                    transformed_x, transformed_y, w = transformed_point
                    # # Normalize the coordinates (divide by w)
                    transformed_x /= w
                    transformed_y /= w
                    x.append(float(transformed_x))
                    y.append(float(transformed_y))

        response.x = x
        response.y = y
        self.get_logger().info("Returning response")
        return response


    def Tobinray(self, img):
        imgray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        ret, binary = cv.threshold(imgray, 127, 255, 0)
        cv.imshow('binary', binary)
        return binary

    def GetContours(self, binary, img):
        contours, hierarchy = cv.findContours(binary, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
        dst = cv.drawContours(img.copy(), contours, -1, (255, 0, 0), 1)
        cv.imshow('dst', dst)

        contour_points = np.concatenate([contour.flatten() for contour in contours])

        # Save the contour points as a NumPy array to a file with two columns
        np.savetxt('contour_points_2columns.txt', contour_points.reshape(-1, 2), fmt='%d')

    def image_callback(self, msg):
        self.get_logger().info('Receivied smile face')
        img = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        binary = self.Tobinray(img)
        self.GetContours(binary, img)


def main(args=None):
    rclpy.init(args=args)
    contour_detection_node = ContourDetectionNode()
    rclpy.spin(contour_detection_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()