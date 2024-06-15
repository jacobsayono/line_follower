import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from std_msgs.msg import Int32

class LineDetector(Node):
    def __init__(self):
        super().__init__('line_detector')
        self.get_logger().info('Line Detector Initialized')
        self.image_sub = self.create_subscription(Image, "/image_in", self.image_callback, 10)
        self.point_pub = self.create_publisher(Point, "/line_position", 10)
        self.processed_image_pub = self.create_publisher(Image, "/processed_image", 1)
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Int32,
            'shutdown_signal',
            self.shutdown_callback,
            10)

    def shutdown_callback(self, msg):
        if msg.data == 1:  # Check if the shutdown signal is received
            self.get_logger().info('Shutdown signal received, shutting down...')
            line_detector.destroy_node()
            rclpy.shutdown()

    def find_line(self, cv_image, draw_image):
        black_threshold = 50
        _, black_mask = cv2.threshold(cv_image, black_threshold, 255, cv2.THRESH_BINARY_INV)
        black_mask = np.uint8(black_mask)

        kernel = np.ones((5, 5), np.uint8)
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, kernel)
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            line_center_x = x + w // 2

            # Draw bounding box in green
            cv2.rectangle(draw_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Draw center line in red
            cv2.line(draw_image, (line_center_x, y), (line_center_x, y + h), (0, 0, 255), 2)

            # Check if the bounding box touches the image edges
            img_width = draw_image.shape[1]
            img_height = draw_image.shape[0]

            if x == 0 and (x + w) == img_width and y > img_height // 4:
                return -2, black_mask  # Intersection
            elif x == 0 and y > 0 and y < img_height // 2:
                return -4, black_mask  # Sharp left turn
            elif (x + w) == img_width and y > 0 and y < img_height // 2:
                return -3, black_mask  # Sharp right turn
            # elif x == 0 and (x + w) == img_width and y == 0 and (y + h) == img_height:  # All black
            #     return -5

            return line_center_x, black_mask  # Normal line detection

        return -1, black_mask  # No line detected (empty space or obstacle)


    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return

        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        scale_percent = 50
        width = int(gray_image.shape[1] * scale_percent / 100)
        height = int(gray_image.shape[0] * scale_percent / 100)
        dim = (width, height)
        resized_gray_image = cv2.resize(gray_image, dim, interpolation=cv2.INTER_AREA)

        crop_start = height // 2
        cropped_image = resized_gray_image[crop_start:height, :]

        # Create a copy for drawing
        drawing_image = cv2.cvtColor(cropped_image, cv2.COLOR_GRAY2BGR)

        # Pass the drawing image for modifications
        line_x, processed_image = self.find_line(cropped_image, drawing_image)

        processed_img_msg = self.bridge.cv2_to_imgmsg(drawing_image, "bgr8")
        self.processed_image_pub.publish(processed_img_msg)

        self.get_logger().info(f"Black line center x-position: {line_x}")
        point_out = Point(x=float(line_x), y=0.0, z=0.0)
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
