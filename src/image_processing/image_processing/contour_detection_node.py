import numpy as np
from custom_messages.srv import PathClient
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformListener
from tf2_ros.buffer import Buffer
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import cv2 as cv
from sklearn.cluster import KMeans

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
        self.pub = self.create_publisher(Bool, '/image_obtained', 10)
        self.bridge = CvBridge()
        self.image_shape = (640, 480)
        self.found_contours = False
        self.notified_processes = False

    def find_homography(self):
        # assuming aruco markers go 1,2,3,4 clockwise. 1 is futhest away from robot base
        # paper is horezontal, simmilar to table.
        source_frame = "paper_corner_0"
        target_frame = "base_link"
        try:
            corner1 = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
        except:
            self.get_logger().info("Can't find canvas 0")
            return

        source_frame = "paper_corner_1"
        target_frame = "base_link"
        try:
            corner2 = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
        except:
            self.get_logger().info("Can't find canvas 1")
            return

        source_frame = "paper_corner_2"
        target_frame = "base_link"
        try:
            corner3 = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
        except:
            self.get_logger().info("Can't find canvas 2")
            return

        source_frame = "paper_corner_3"    
        target_frame = "base_link"
        try:
            corner4 = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
        except:
            self.get_logger().info("Can't find canvas 3")
            return


        # need to check if these produce messurments in the right sign. Could have gotten confuesed with axies orientations.
        paper_hight = corner1.transform.translation.x - corner3.transform.translation.x
        paper_lenght = corner1.transform.translation.x - corner2.transform.translation.x
        paper_ratio = paper_hight / paper_lenght

        # will add "crop" from camera length resolution "x" variable.
        # this will stop lines from warping during transfromation by keeping the hight/lenght ratio the same as the paper
        cam_height = self.image_shape[1]
        cam_length = self.image_shape[0]
        
        dst_points = np.array([
            [corner1.transform.translation.x, corner1.transform.translation.y],  # Point 1
            [corner2.transform.translation.x, corner2.transform.translation.y],  # Point 2
            [corner3.transform.translation.x, corner3.transform.translation.y],  # Point 3
            [corner4.transform.translation.x, corner4.transform.translation.y]   # Point 4
        ], dtype=np.float32)
        # Define the destination points (desired points)
        src_points = np.array([
            [-75, 0], 
            [-75, cam_height],
            [cam_length + 75, cam_height],  # New position for Point 3
            [cam_length + 75, 0], 
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
        with open("points" + str(index) + ".txt", "r") as file:
            # Read each line in the file
            for line in file:
                # Split each line into two values using a space as the delimiter
                values = line.split(', ')
                if len(values) == 3:
                    x_value, y_value, z_value = map(float, values)
                    transformed_point = np.dot(self.H, [x_value, y_value, 1])

                    # Access the transformed coordinates
                    transformed_x, transformed_y, w = transformed_point
                    # # Normalize the coordinates (divide by w)
                    transformed_x /= w
                    transformed_y /= w
                    x.append(float(transformed_x))
                    y.append(float(transformed_y))
                    z.append(float(z_value))

        response.x = x
        response.y = y
        response.z = z
        self.get_logger().info("Returning response")
        return response
    
    def find_contours(self, index, cluster_labels, image):
        segmented_image = np.zeros_like(image)

        # Segmented Image for Label
        colors = {
            0: [0, 0, 255],  # Color for label 0
            1: [0, 255, 0],  # Color for label 1
            2: [255, 0, 0],  # Color for label 2
            3: [0, 255, 255],  # Color for label 3
            4: [255, 255, 0],  # Color for label 4
            5: [255, 0, 255],  # Color for label 5
        }

        # Create a white background image
        white_background = np.ones_like(image) * 255

        label_to_show = index  # Specify the label to show (0 in this case)

        for label in np.unique(cluster_labels):
            mask = (cluster_labels == label)
            if label == label_to_show:
                color = colors[label]
                segmented_image[mask.reshape(image.shape[:-1])] = color

        output_image = np.ones_like(image) * 255
   
        for label in np.unique(cluster_labels):
            mask = (cluster_labels == label)
            if label == label_to_show:
                color = colors[label]
                output_image[mask.reshape(image.shape[:-1])] = color

        contours, _ = cv.findContours(cv.cvtColor(segmented_image, cv.COLOR_BGR2GRAY), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)


        # Simplify the contours using the Douglas-Peucker algorithm
        simplified_contours = []
        epsilon = 5  # Adjust the value of epsilon as needed for desired simplification
        min_contour_area = 5

        for contour in contours:
            contour_area = cv.contourArea(contour)
            if contour_area > min_contour_area:
                simplified_contour = cv.approxPolyDP(contour, epsilon, closed=True)
                simplified_contours.append(simplified_contour)

        # Extract contour points from simplified contours
        contour_points = []
        for contour in simplified_contours:
            x, y = contour[0][0]
            z = 0.2
            contour_points.append((x, y, z))
            for point in contour:
                x, y = point[0]  # Extract x and y coordinates
                z = 0
                contour_points.append((x, y, z))
            z = 0.2
            contour_points.append((x, y, z))

        # Save contour points to a text file
        with open('points' + str(index) + '.txt', 'w') as file:
            for x, y, z in contour_points:
                file.write(f'{x}, {y}, {z}\n')
        if not self.notified_processes:
            msg = Bool()
            self.pub.publish(msg)
            self.notified_processes = True


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
        if self.found_contours: return
        self.found_contours = True
        # Convert the ROS Image message to an OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        b, g, r = cv.split(cv_image)
        self.image_shape = cv_image.shape
        b = b.flatten()
        g = g.flatten()
        r = r.flatten()

        color_samples = np.column_stack((r, g, b))

        n_clusters = 5

        kmeans = KMeans(n_clusters=n_clusters, random_state=0)
        cluster_labels = kmeans.fit_predict(color_samples)

        i = 0
        while(i < n_clusters):
            self.find_contours(i, cluster_labels, cv_image)
            i += 1


def main(args=None):
    rclpy.init(args=args)
    contour_detection_node = ContourDetectionNode()
    rclpy.spin(contour_detection_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()