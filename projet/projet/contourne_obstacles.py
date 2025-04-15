import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

class ContourneObstacles(Node):
    def __init__(self):
        super().__init__('contourne_obstacles_node')

        # Déclare et récupère le paramètre 'distance_limit'
        self.declare_parameter('distance_limit', 0.20)

        self.distance_limit = self.get_parameter('distance_limit').get_parameter_value().double_value

        # Crée un abonné au sujet 'lds_distances'
        self.subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.contourne_callback,
            10
        )

        # Crée un éditeur pour le sujet '/cmd_vel'
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("Arrêt automatique activé...")

    def contourne_callback(self, msg):
        distances = msg.ranges
        self.get_logger().info(f"[DEBUG] Nombre de distances : {len(distances)}")

        for i in range (20, 55) :   
            if distances[i] < self.distance_limit:
                contourne_msg = Twist()
                contourne_msg.linear.x = 0.02
                contourne_msg.angular.z = -1.1
                self.publisher.publish(contourne_msg)
                self.get_logger().info("Obstacle à gauche détecté ! On contourne par la droite.")
                self.get_logger().info(f" indice de distance = {i}")

        for i in range (280, 341) :   
            if distances[i] < self.distance_limit:
                contourne_msg = Twist()
                contourne_msg.linear.x = 0.02
                contourne_msg.angular.z = 0.5
                self.publisher.publish(contourne_msg)
                self.get_logger().info("Obstacle à droite détecté ! On contourne par la gauche.")
                self.get_logger().info(f" indice de distance = {i}")


        if not distances:
            self.get_logger().warn("⚠️ Aucun obstacle détecté : distances vides !")

def main(args=None):
    rclpy.init(args=args)
    node = ContourneObstacles()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
