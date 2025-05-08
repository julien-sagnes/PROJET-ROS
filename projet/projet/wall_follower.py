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

        # Très grande valeurs arbitraires
        self.front_dist = 10.0
        self.left_dist = 10.0
        self.right_dist = 10.0

        # PI
        self.kp = 1.0
        self.ki = 0.1
        self.integral = 0.0
        self.coef = 1.0
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
        
        # Retour en ligne droite (réinitialisation du PID progressif)
        if self.front_dist > 0.8 or math.isinf(self.front_dist):
            self.coef -= 0.008  # à modifier avec test sur robot réel
            self.get_logger().warn("Détection de ligne droite.")
            self.integral *= self.coef

        # Mur perdu : avance lentement
        correction_add = 0.05
        if abs(self.right_dist) > 0.3 or math.isinf(self.right_dist):
            self.get_logger().warn("Mur perdu !")
            twist.linear.x = self.linear_speed
            twist.angular.z = - correction_add
        elif abs(self.left_dist) > 0.3 or math.isinf(self.left_dist):
            self.get_logger().warn("Mur perdu !")
            twist.linear.x = self.linear_speed
            twist.angular.z = + correction_add

        else:
            self.coef = 1.0

            error = self.left_dist - self.right_dist

            # Calcul du temps écoulé
            current_time = time.time()
            dt = current_time - self.previous_time
            self.previous_time = current_time

            # Mise à jour de l'intégrale
            self.integral += error * dt

            correction = self.kp * error + self.ki * self.integral

            twist.linear.x = self.linear_speed
            twist.angular.z = correction

            self.get_logger().info(
            f"Front: {self.front_dist:.2f}, Right: {self.right_dist:.3f}, Left: {self.left_dist}, "
            f"Proportionnal: {self.kp * error:.3f}, Integral: {(self.ki * self.integral):.3f}, Angular Z: {twist.angular.z:.3f}")

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()