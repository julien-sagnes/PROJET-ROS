import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import math

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')

        self.subscriber = self.create_subscription(
            Float32MultiArray,
            '/lds_distances',
            self.dist_callback,
            10
        )

        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.declare_parameter('linear_speed', 0.05)
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value

        self.front_dist = float('inf')
        self.left_dist = float('inf')
        self.right_dist = float('inf')

        # PID
        self.kp = 7.0
        self.ki = 0.4
        self.kd = 1.0
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = self.get_clock().now().nanoseconds / 1e9

        self.get_logger().info("Wall follower node started.")

    def dist_callback(self, msg):
        if len(msg.data) >= 3:
            self.front_dist = msg.data[0]
            self.left_dist = msg.data[1]
            self.right_dist = msg.data[2]
            self.control_loop()

    def control_loop(self):
        twist = Twist()

        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time
        dt = max(dt, 1e-4)  # éviter division par 0

        if any(d in [float('inf'), float('-inf')] for d in [self.left_dist, self.right_dist]) or self.left_dist > 0.18 or self.right_dist > 0.18:
            self.get_logger().warn("Mur perdu ! Recherche...")
            twist.linear.x = self.linear_speed / 2
            twist.angular.z = -0.1

        else:
            error = self.left_dist - self.right_dist
            self.integral += error * dt
            derivative = (error - self.last_error) / dt

            correction = (
                self.kp * error +
                self.ki * self.integral +
                self.kd * derivative
            )

            twist.linear.x = self.linear_speed
            twist.angular.z = correction

            self.last_error = error

            self.get_logger().info(
                f"Left: {self.left_dist:.3f}, Right: {self.right_dist:.3f}, Error: {error:.3f}, Angular Z: {twist.angular.z:.3f}")

        self.publisher.publish(twist)

        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()