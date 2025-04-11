import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import time

class AutomaticStop(Node):
    def __init__(self):
        super().__init__('automatic_stop_node')

        # Déclare et récupère le paramètre 'distance_limit'
        self.declare_parameter('distance_limit', 0.15)

        self.distance_limit = self.get_parameter('distance_limit').get_parameter_value().double_value

        # Crée un abonné au sujet 'lds_distances'
        self.subscriber = self.create_subscription(
            Float32MultiArray,
            'lds_distances',
            self.lds_callback,
            10
        )

        # Crée un éditeur pour le sujet '/cmd_vel'
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("Arrêt automatique activé...")

    def lds_callback(self, msg):
        # Vérifie les distances et ajuste la correction si nécessaire
        if msg.data[0] < self.distance_limit:
            correction_needed = True
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.publisher.publish(stop_msg)
            self.get_logger().info("Obstacle détecté à l'avant ! Arrêt.")

def main(args=None):
    rclpy.init(args=args)
    node = AutomaticStop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
