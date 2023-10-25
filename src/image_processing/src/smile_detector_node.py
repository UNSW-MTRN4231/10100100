import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class SmileDetectionNode(Node):
    def __init__(self):
        super().__init__('smile_detection_node')
        self.publisher = self.create_publisher(Image, '/image_raw', 10)
        self.subscriber = self.create_subscription(String, '/commands', self.handle_command, 10)
        self.cv_bridge = CvBridge()
        self.face_cascade = cv2.CascadeClassifier('./haarcascade_frontalface_default.xml')
        self.smile_cascade = cv2.CascadeClassifier('./haarcascade_smile.xml')
        self.counter = 0
        self.smile_detected = False
        self.subscriber
        self.video_capture = cv2.VideoCapture(0)
        self.timer = self.create_timer(0.1, self.timer_callback)
    
    def handle_command(self, msg):
        print(msg.data)
        if (msg.data == 'restart'):
            self.smile_detected = False

    def detect(self, gray):
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
        for (x, y, w, h) in faces:
            roi_gray = gray[y:y + h, x:x + w]
            smiles = self.smile_cascade.detectMultiScale(roi_gray, 1.8, 20)
            if len(smiles) > 0:
                return True
        return False
    
    def timer_callback(self):
        if self.video_capture.isOpened():
            _, frame = self.video_capture.read()
            cv2.imshow('Video', frame)
            if (self.smile_detected):
                ros_image = self.cv_bridge.cv2_to_imgmsg(self.current_image, encoding="bgr8")
                self.publisher.publish(ros_image)
                
            else:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                smile = self.detect(gray)
                if smile:
                    self.counter += 1
                    if self.counter > 10:
                        # Publish the image to the /image_raw topic
                        self.current_image = frame
                        self.smile_detected = True
                        print("detected smile")
        

    def __del__(self):
        self.video_capture.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    smile_detection_node = SmileDetectionNode()
    rclpy.spin(smile_detection_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()