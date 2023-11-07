
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from sklearn.cluster import KMeans

class ImageSegmentationServer(Node):

    def __init__(self):
        super().__init__('image_segmentation_server')
        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(
            Image,
            'input_image',# replace the node name
            self.image_callback,
            10
        )
        self.label_subscriber = self.create_subscription(
            Int32,
            'label_to_show',# replace the node name
            self.label_callback,
            10
        )
        self.segmented_image_publisher = self.create_publisher(
            Image,
            'segmented_image',
            10
        )
        self.label_to_show = 0
        self.image = None

        def save_contour_points(self, contour_points):
            filename = 'simplified_contour_points.txt'
            with open(filename, 'w') as file:
                for x, y in contour_points:
                    file.write(f'{x}, {y}\n')

        def segment_image(self):
            if self.image is not None:
                b, g, r = cv2.split(self.image)
                b = b.flatten()
                g = g.flatten()
                r = r.flatten()
                color_samples = np.column_stack((r, g, b))
                n_clusters = 6
                kmeans = KMeans(n_clusters=n_clusters, random_state=0)
                cluster_labels = kmeans.fit_predict(color_samples)

                segmented_image = np.zeros_like(self.image)

                colors = {
                    0: [0, 0, 255],
                    1: [0, 255, 0],
                    2: [255, 0, 0],
                    3: [0, 255, 255],
                    4: [255, 255, 0],
                    5: [255, 0, 255],
                }

                output_image = np.ones_like(self.image) * 255

                for label in np.unique(cluster_labels):
                    mask = (cluster_labels == label)
                    if label == self.label_to_show:
                        color = colors[label]
                        output_image[mask.reshape(self.image.shape[:-1])] = color

                # Convert the output image to a ROS2 Image message
                segmented_image_msg = self.bridge.cv2_to_imgmsg(output_image, encoding='bgr8')
                self.segmented_image_publisher.publish(segmented_image_msg)

                contours, _ = cv2.findContours(cv2.cvtColor(segmented_image, cv2.COLOR_BGR2GRAY), cv2.RETR_EXTERNAL,
                                               cv2.CHAIN_APPROX_SIMPLE)
                simplified_contours = []
                epsilon = 5
                min_contour_area = 5

                for contour in contours:
                    contour_area = cv2.contourArea(contour)
                    if contour_area > min_contour_area:
                        simplified_contour = cv2.approxPolyDP(contour, epsilon, closed=True)
                        simplified_contours.append(simplified_contour)

                contour_points = []
                for contour in simplified_contours:
                    for point in contour:
                        x, y = point[0]
                        contour_points.append((x, y))

                self.save_contour_points(contour_points)

    def main(args=None):
        rclpy.init(args=args)
        node = ImageSegmentationServer()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()