import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
# from geometry_msgs.msg import TwistStamped
# from std_msgs.msg import Header
from std_msgs.msg import Int32
# import time

class FollowLine(Node):
    def __init__(self):
        super().__init__('follow_line')
        self.subscription = self.create_subscription(
            Point,
            '/line_position',
            self.line_callback,
            10)
        # self.declare_parameter("base_link", "")
        # self.frame_id = str(self.get_parameter("base_link").value)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_tracker', 10)
        # self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel_tracker', 10)
        self.get_logger().info('Follow Line Node Started')
        self.shutdown_publisher = self.create_publisher(Int32, 'shutdown_signal', 10)
        # self.timer = self.create_timer(5, self.publish_signal)  # Publish every 5 seconds

    def line_callback(self, msg):
        twist = Twist()
        # commands = TwistStamped()
        center_x = msg.x

        if center_x >= 0:
            image_center_x = 160  # Assuming image width of 640
            error = center_x - image_center_x
            twist.linear.x = 0.375  # Constant forward speed
            twist.angular.z = -0.01 * error  # Proportional control
        elif center_x == -4:  # sharp left turn
            twist.linear.x = 0.5
            twist.angular.z = 2.5
        elif center_x == -3:  # sharp right turn
            twist.linear.x = 0.5
            twist.angular.z = -2.5
        elif center_x == -2:  # intersection
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info('Intersection detected, shutting down node.')
            msg = Int32()
            msg.data = 1
            self.shutdown_publisher.publish(msg)
            follow_line.destroy_node()
            rclpy.shutdown()
        # elif center_x == -5:  # all black so at pickup/dropoff
            # subprocess.run(['sudo', '/home/pi/pickup.sh'], check=True)
        elif center_x == -1:  # No line detected (no box)
            twist.linear.x = -0.3
            twist.angular.z = 0.0
            # commands.header.stamp = self.get_clock().now().to_msg()
            # commands.header.frame_id = self.frame_id
            # commands.twist.linear.x = 0.5
            # commands.twist.angular.z = 4.0
            # self.publisher_.publish(twist)
            # self.get_logger().info(f'Published cmd_vel: linear={commands.twist.linear.x}, angular={commands.twist.angular.z}')


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
