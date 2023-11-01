import numpy as np
from custom_messages.srv import PathClient

class ImageProcessingNode(Node):

    def __init__(self):
        super().__init__('contour_detection_node')
        self.subscription = self.create_subscription(
            Image,
            'image_topic',  # replace the former node name
            self.image_callback,
            10)
        self.pubscription
        self.srv = self.create_service(PathClient, 'path_client', self.callback)

    def callback(self, request, response):
        # Process the request containing an array of two integers
        x, y = request.data
        self.get_logger().info(f"Received request with data: ({x}, {y})")

        # Create an example response with an array of points (OpenCV contours)
        contour = np.array([[x, y], [x + 10, y + 10], [x + 20, y + 20]], np.int32)
        response.points.append(Point(x=x, y=y))
        response.points.append(Point(x=x+10, y=y+10))
        response.points.append(Point(x=x+20, y=y+20))
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
    image_processing_node = ImageProcessingNode()
    rclpy.spin(image_processing_node)
    image_processing_node.destroy_node()
    rclpy.shutdown()