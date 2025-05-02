import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class ContourneObstacles(Node):
    def __init__(self):
        super().__init__('contourne_obstacles_node')

        self.declare_parameter('distance_limit', 0.35)
        self.distance_limit = self.get_parameter('distance_limit').get_parameter_value().double_value

        self.subscriber = self.create_subscription(LaserScan, '/scan', self.contourne_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("Node 'contourne_obstacles' activé...")

    def contourne_callback(self, msg):
        distances = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        # Afficher uniquement les mesures valides (ni 0.0 ni inf)
        for i, dist in enumerate(distances):
            if dist != 0.0 and not math.isinf(dist):
                angle = angle_min + i * angle_increment
                self.get_logger().debug(f"Index {i} - Angle: {math.degrees(angle):.1f}° - Distance: {dist:.2f} m")

        # Zones d'analyse gauche et droite
        gauche_detecte = False
        droite_detecte = False

        # Zone gauche : indices 20 à 54
        for i in range(1, 80):
            dist = distances[i]
            if 0.0 < dist < self.distance_limit:
                gauche_detecte = True
                self.get_logger().info(f"Obstacle à gauche (index {i}) : {dist:.2f} m")
                break  # On agit dès le premier obstacle

        # Zone droite : indices 280 à 340
        for i in range(280, 359):
            dist = distances[i]
            if 0.0 < dist < self.distance_limit:
                droite_detecte = True
                self.get_logger().info(f"Obstacle à droite (index {i}) : {dist:.2f} m")
                break  # On agit dès le premier obstacle

        # Action
        cmd = Twist()
        if gauche_detecte:
            cmd.linear.x = 0.02
            cmd.angular.z = -0.5  # Tourne à droite
            self.publisher.publish(cmd)
            self.get_logger().info("Contournement par la droite")
        elif droite_detecte:
            cmd.linear.x = 0.02
            cmd.angular.z = 0.5  # Tourne à gauche
            self.publisher.publish(cmd)
            self.get_logger().info("Contournement par la gauche")
        else:
            # Si pas d'obstacle : avancer tout droit ou ne rien faire
            self.get_logger().info("Aucun obstacle détecté dans les zones critiques")

def main(args=None):
    rclpy.init(args=args)
    node = ContourneObstacles()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
