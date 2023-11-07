import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

class CameraInfoPublisher(Node):

    def __init__(self):
        super().__init__('camera_info_publisher')
        self.publisher = self.create_publisher(CameraInfo, '/camera_info', 10)

        # Define the camera info parameters
        self.width = 640  # Set the image width
        self.height = 480  # Set the image height
        self.camera_name = 'narrow_stereo'  # Set the camera name
        self.distortion_model = 'plumb_bob'  # Set the distortion model
        self.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # Set distortion coefficients
        self.k = [525.0, 0.0, 320.0, 0.0, 525.0, 240.0, 0.0, 0.0, 1.0]  # Set camera matrix (intrinsic)
        self.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]  # Set rotation matrix
        self.p = [525.0, 0.0, 320.0, 0.0, 0.0, 525.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]  # Set projection matrix
        self.binning_x = 0  # Set binning along the image x-axis
        self.binning_y = 0  # Set binning along the image y-axis
        self.timer = self.create_timer(1, self.publish_camera_info) 

    def publish_camera_info(self):
        msg = CameraInfo()
        msg.width = self.width
        msg.height = self.height
        msg.distortion_model = self.distortion_model
        msg.d = self.d
        msg.k = self.k
        msg.r = self.r
        msg.p = self.p
        msg.binning_x = self.binning_x
        msg.binning_y = self.binning_y
        msg.header.frame_id = self.camera_name
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)
        self.get_logger().info('Camera info published to camera_info')

def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()