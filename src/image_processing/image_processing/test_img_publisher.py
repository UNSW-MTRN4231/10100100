import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class TestImagePublisher(Node):

    def __init__(self):
        super().__init__('test_image_publisher')
        self.publisher = self.create_publisher(Image, '/image_raw', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1, self.publish_image)

    def publish_image(self):
        try:
            image = cv2.imread('aruco_image.jpg')  # Load the image from the file
            if image is not None:
                image_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
                self.publisher.publish(image_msg)
                self.get_logger().info('Image published to /image_raw')
            else:
                self.get_logger().warn('Image not found')
        except Exception as e:
            self.get_logger().error(f'Error publishing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = TestImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()