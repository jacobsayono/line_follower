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
        # Threshold value for detecting black
        black_threshold = 50  # Adjust this threshold based on your specific lighting conditions and camera settings

        # Threshold the grayscale image to get a binary mask
        _, black_mask = cv2.threshold(cv_image, black_threshold, 255, cv2.THRESH_BINARY_INV)

        # Ensure the mask is in the correct type, should be uint8
        black_mask = np.uint8(black_mask)

        # Clean up image using morphological operations
        kernel = np.ones((5, 5), np.uint8)
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
            # Convert ROS image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return

        # Convert to grayscale
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Resize the grayscale image to reduce resolution and processing time
        scale_percent = 50  # percentage of original size
        width = int(gray_image.shape[1] * scale_percent / 100)
        height = int(gray_image.shape[0] * scale_percent / 100)
        dim = (width, height)
        resized_gray_image = cv2.resize(gray_image, dim, interpolation=cv2.INTER_AREA)

        # Crop the image to the lower third
        crop_start = 2 * height // 3  # Start cropping at two-thirds down the image
        cropped_image = resized_gray_image[crop_start:height, :]

        # Publish the original image for visualization
        original_img_msg = self.bridge.cv2_to_imgmsg(resized_gray_image, "mono8")
        self.original_image_pub.publish(original_img_msg)

        # Process the cropped grayscale image to find the line
        line_x, processed_image = self.find_line(cropped_image)

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
