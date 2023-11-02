import numpy as np
from custom_messages.srv import PathClient
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2 as cv

class ContourDetectionNode(Node):

    def __init__(self):
        super().__init__('contour_detection_node')
        self.subscription = self.create_subscription(
            Image,
            'person_snapshot',  # replace the former node name
            self.image_callback,
            10)

    
        self.srv = self.create_service(PathClient, 'path_service', self.callback)

    def callback(self, request, response):
        # Process the request containing an array of two integers
        index = request.colour[0]
        # Initialize empty lists for x and y values
        x = []
        y = []
        self.get_logger().info("Here ya dog")
        # Open the text file for reading
        with open("test.txt", "r") as file:
            # Read each line in the file
            for line in file:
                # Split each line into two values using a space as the delimiter
                values = line.split()
                if len(values) == 2:
                    x_value, y_value = map(int, values)
                    x.append(x_value)
                    y.append(y_value)
        response.x = x
        response.y = y
        response.width = 1080
        response.height = 720
        self.get_logger().info(str(response))
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