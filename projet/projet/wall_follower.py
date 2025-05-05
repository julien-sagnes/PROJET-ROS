import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import time
import math

from std_msgs.msg import Float32MultiArray

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower_node')

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

        self.start_time = time.time()
        self.wait_time = 0.0    # Nombre de secondes avant de bouger le robot

        self.declare_parameter('linear_speed', 0.025)
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value

        # Très grande valeurs arbitraires
        self.front_dist = 10.0
        self.left_dist = 10.0
        self.right_dist = 10.0

        # P
        self.kp = 5.5

        self.get_logger().info("Wall follower node started.")

    def dist_callback(self, msg):
        if len(msg.data) >= 3:
            self.front_dist = msg.data[0]
            self.left_dist = msg.data[1]
            self.right_dist = msg.data[2]
            self.control_loop()

    def control_loop(self):
        twist = Twist()
        now = time.time()

        if abs(now - self.start_time) < self.wait_time:
            self.get_logger().info(f"Attente durant {now - self.start_time} / {self.wait_time}.")
            return
    
        # Virage à gauche
        if abs(self.left_dist) > 10 or math.isinf(self.left_dist):
            self.get_logger().warn("Correction à droite...")
            twist.linear.x = self.linear_speed
            twist.angular.z = - 0.1

        # Virage à droite
        elif self.right_dist > 10 or math.isinf(self.right_dist):
            self.get_logger().warn("Correction à gauche...")
            twist.linear.x = self.linear_speed
            twist.angular.z = + 0.1

        else:
            error = self.left_dist - self.right_dist

            correction = self.kp * error

            twist.linear.x = self.linear_speed
            twist.angular.z = correction

            self.get_logger().info(
                f"Front : {self.front_dist}, Left: {self.left_dist:.3f}, Right: {self.right_dist:.3f}, Error: {error:.3f}, Angular Z: {twist.angular.z:.3f}")

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()