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

        self.declare_parameter('linear_speed', 0.05)
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value

        # Très grandes valeurs arbitraires
        self.front_dist = 10.0
        self.left_dist = 10.0
        self.right_dist = 10.0

        # PI
        self.kp = 4.2
        self.previous_time = time.time()

        self.get_logger().info("Wall follower node started.")

    def dist_callback(self, msg):
        if len(msg.data) >= 3:
            self.front_dist = msg.data[0]
            self.left_dist = msg.data[1]
            self.right_dist = msg.data[2]
            self.back_dist = msg.data[3]
            self.control_loop()

    def control_loop(self):
        twist = Twist()
        now = time.time()

        if abs(now - self.start_time) < self.wait_time:
            self.get_logger().info(f"Attente durant {now - self.start_time} / {self.wait_time}.")
            return

        # Mur perdu : avance lentement
        tolerance = 0.3
        correc_add = 0.05
        if abs(self.right_dist) > tolerance or math.isinf(self.right_dist):
            self.get_logger().warn("Mur droit perdu !")
            twist.linear.x = self.linear_speed
            twist.angular.z = - correc_add
        elif abs(self.left_dist) > tolerance or math.isinf(self.left_dist):
            self.get_logger().warn("Mur gauche perdu !")
            twist.linear.x = self.linear_speed
            twist.angular.z = + correc_add


        else:
            self.coef = 1.0

            error = self.left_dist - self.right_dist

            correction = self.kp * error
            twist.linear.x = self.linear_speed
            twist.angular.z = correction

            self.get_logger().info(
            f"Front: {self.front_dist:.2f}, Right: {self.right_dist:.3f}, Left: {self.left_dist}, "
            f"Proportionnal: {self.kp * error:.3f}, Angular Z: {twist.angular.z:.3f}")

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()