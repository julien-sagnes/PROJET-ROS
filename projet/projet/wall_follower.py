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
        self.kp = 8.0
        self.ki = 0.4
        self.kd = 1.0
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = self.get_clock().now().nanoseconds / 1e9

        # Gérer les dispartions de murs
        self.lost_wall_timeout = 0.8
        self.last_wall_seen_time = self.last_time

        self.search_direction = 1  # 1 = gauche, -1 = droite
        self.search_phase_start = self.get_clock().now().nanoseconds / 1e9
        self.search_duration = 1.0  # durée initiale de chaque phase (sec)

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

        # Cas de perte du mur gauche
        if self.left_dist > 0.2:
            time_since_wall = current_time - self.last_wall_seen_time

            if time_since_wall > self.lost_wall_timeout:
                twist.linear.x = 0.0

                # Temps depuis le début de cette phase de rotation
                time_in_phase = current_time - self.search_phase_start

                # Si on dépasse la durée prévue, on change de direction
                if time_in_phase > self.search_duration:
                    self.search_direction *= -1  # changer de sens
                    self.search_phase_start = current_time

                    if self.search_direction == 1:
                        # à chaque retour à gauche, on augmente la durée
                        self.search_duration += 1.0

                twist.angular.z = 0.4 * self.search_direction  # tourner à gauche ou droite

                direction_str = "gauche" if self.search_direction == 1 else "droite"
                self.get_logger().warn(f"Mur perdu : recherche vers la {direction_str} pendant {self.search_duration:.1f}s")

                self.publisher.publish(twist)
                return
        else:
            self.last_wall_seen_time = current_time

        if any(d in [float('inf'), float('-inf')] for d in [self.left_dist, self.right_dist, self.front_dist]):
            self.get_logger().warn("Données LIDAR incomplètes...")

        else:
            # Suivi de mur latéral
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

            self.get_logger().info(f"Left: {self.left_dist:.3f}, Right: {self.right_dist:.3f}, Error: {error:.3f}, Angular Z: {twist.angular.z:.3f}")

            self.publisher.publish(twist)

            self.last_error = error
            self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()