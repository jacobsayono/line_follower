import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class LineDetector(Node):
    def __init__(self):
        super().__init__('line_detector')
        self.get_logger().info('Line Detector Initialized')
        self.image_sub = self.create_subscription(Image, "/image_in", self.image_callback, 10)
        self.point_pub = self.create_publisher(Point, "/line_position", 10)
        self.original_image_pub = self.create_publisher(Image, "/original_image", 1)
        self.processed_image_pub = self.create_publisher(Image, "/processed_image", 1)
        self.bridge = CvBridge()

    def find_line(self, cv_image):
        # Define HSV range for black and white
        white_range = {'h_min': 0, 'h_max': 180, 's_min': 0, 's_max': 50, 'v_min': 200, 'v_max': 255}
        black_range = {'h_min': 0, 'h_max': 180, 's_min': 0, 's_max': 255, 'v_min': 0, 'v_max': 50}

        # Convert image to HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Thresholding for white and black
        white_mask = cv2.inRange(hsv_image, (white_range['h_min'], white_range['s_min'], white_range['v_min']),
                                          (white_range['h_max'], white_range['s_max'], white_range['v_max']))
        black_mask = cv2.inRange(hsv_image, (black_range['h_min'], black_range['s_min'], black_range['v_min']),
                                          (black_range['h_max'], black_range['s_max'], black_range['v_max']))

        # Clean up image using morphological operations
        kernel = np.ones((5,5), np.uint8)
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, kernel)
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel)

        # Find contours in the black mask
        contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Find the largest contour, assuming it's the line
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            line_center_x = x + w // 2
            return line_center_x, black_mask

        return None, black_mask

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return

        # Publish the original image for visualization
        original_img_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.original_image_pub.publish(original_img_msg)

        line_x, processed_image = self.find_line(cv_image)

        # Publish the processed image
        processed_img_msg = self.bridge.cv2_to_imgmsg(processed_image, "mono8")
        self.processed_image_pub.publish(processed_img_msg)

        if line_x is None:
            line_x = -1  # Indicate no line detected

        self.get_logger().info(f"Black line center x-position: {line_x}")
        point_out = Point(x=float(line_x), y=0.0, z=0.0)  # y, z are not used
        self.point_pub.publish(point_out)

def main(args=None):
    rclpy.init(args=args)
    line_detector = LineDetector()
    rclpy.spin(line_detector)

    line_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# ros2 run rqt_image_view rqt_image_view
