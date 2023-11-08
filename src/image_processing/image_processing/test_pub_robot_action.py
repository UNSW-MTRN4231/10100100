import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from custom_messages.msg import RobotAction

class CameraInfoPublisher(Node):

    def __init__(self):
        super().__init__('camera_info_publisher')
        self.publisher = self.create_publisher(RobotAction, '/robot_action', 10)

        
        self.timer = self.create_timer(1, self.publish_camera_info) 

    def publish_camera_info(self):
        msg = RobotAction()
        msg.command = 'draw'
        msg.x = [0.5]
        msg.y = [0.5]
        msg.z = [0.5]
        self.publisher.publish(msg)
        self.get_logger().info('postion')

def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()