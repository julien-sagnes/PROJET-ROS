import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import numpy as np
import math

class LDSDistanceNode(Node):
    def __init__(self):
        super().__init__('lds_distance_node')
        self.declare_parameter('angle_width_deg', 20.0)  # Paramètre déclarable dans launch ou CLI
        self.angle_width_deg = self.get_parameter('angle_width_deg').get_parameter_value().double_value
        self.angle_width_rad = math.radians(self.angle_width_deg / 2.0)  # Moitié de l'angle de vision

        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.publisher = self.create_publisher(Float32MultiArray, '/lds_distances', 10)

        self.get_logger().info(f'Lidar allumé avec un angle d\'intervalle de {self.angle_width_deg}°')

    def scan_callback(self, msg):
        front_distances = []
        left_distances = []
        right_distances = []
        back_distances = []

        angle_increment = msg.angle_increment
        ranges = msg.ranges

        for i, range_value in enumerate(ranges):
            angle = msg.angle_min + i * angle_increment
            angle = angle % (2 * math.pi)  # S'assurer que l'angle est entre 0 et 2π

            if self.is_in_sector(angle, 0.0):                  # Avant
                front_distances.append(range_value)
            elif self.is_in_sector(angle, math.pi / 2):        # Gauche
                left_distances.append(range_value)
            elif self.is_in_sector(angle, math.pi):            # Arrière
                back_distances.append(range_value)
            elif self.is_in_sector(angle, 3 * math.pi / 2):    # Droite
                right_distances.append(range_value)

        distances_msg = Float32MultiArray()
        distances_msg.data = [
            self.calculate_mean(front_distances),
            self.calculate_mean(left_distances),
            self.calculate_mean(right_distances),
            self.calculate_mean(back_distances)
        ]
        self.publisher.publish(distances_msg)

    def is_in_sector(self, angle, center):
        # Gère les bords circulaires (autour de 0 et 2π)
        delta = self.angle_width_rad
        min_angle = (center - delta) % (2 * math.pi)
        max_angle = (center + delta) % (2 * math.pi)

        if min_angle < max_angle:
            return min_angle <= angle <= max_angle
        else:
            return angle >= min_angle or angle <= max_angle

    def calculate_mean(self, distances):
        valid = [d for d in distances if not math.isinf(d) and not math.isnan(d)]
        if valid:
            return sum(valid) / len(valid)
        return float('inf')

def main(args=None):
    rclpy.init(args=args)
    node = LDSDistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
