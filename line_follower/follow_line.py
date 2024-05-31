import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist

class FollowLine(Node):
    def __init__(self):
        super().__init__('follow_line')
        self.subscription = self.create_subscription(
            Point,
            '/line_position',
            self.line_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Follow Line Node Started')

    def line_callback(self, msg):
        twist = Twist()
        center_x = msg.x
        if center_x == -1:  # No line detected
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            image_center_x = 320  # Assuming image width of 640
            error = center_x - image_center_x
            twist.linear.x = 0.5  # Constant forward speed
            twist.angular.z = -0.005 * error  # Proportional control

        self.publisher_.publish(twist)
        self.get_logger().info(f'Published cmd_vel: linear={twist.linear.x}, angular={twist.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    follow_line = FollowLine()
    rclpy.spin(follow_line)
    follow_line.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
