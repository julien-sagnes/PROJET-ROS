import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import numpy as np

class LDSDistanceNode(Node):
    def __init__(self):
        super().__init__('lds_distance_node')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.publisher = self.create_publisher(Float32MultiArray, '/lds_distances', 10)

        self.get_logger().info(f'Lidar allumé')

    def scan_callback(self, msg):
        front_distances = []
        left_distances = []
        right_distances = []
        back_distances = []

        angle_increment = msg.angle_increment
        ranges = msg.ranges

        for i, range_value in enumerate(ranges):
            angle = msg.angle_min + i * angle_increment
            if -0.349 <= angle <= 0.349:  # ±20 deg in radians
                front_distances.append(range_value)
            elif 1.221 <= angle <= 1.920:  # 70 to 110 deg in radians
                left_distances.append(range_value)
            elif 4.363 <= angle <= 5.061:  # 250 to 290 deg in radians
                right_distances.append(range_value)
            elif (2.879 <= angle <= 3.455) or (-3.455 <= angle <= -2.879):  # 160 to 200 deg in radians
                back_distances.append(range_value)

        front_mean = self.calculate_mean(front_distances)
        left_mean = self.calculate_mean(left_distances)
        right_mean = self.calculate_mean(right_distances)
        back_mean = self.calculate_mean(back_distances)

        distances_msg = Float32MultiArray()
        distances_msg.data = [front_mean, left_mean, right_mean, back_mean]
        self.publisher.publish(distances_msg)
        #self.get_logger().info(f"front = {distances_msg.data[0]}, left = {distances_msg.data[1]}, right = {distances_msg.data[2]}, back = {distances_msg.data[3]}")

    def calculate_mean(self, distances):
        if distances:
            return sum(distances) / len(distances)
        return float('inf')  # Return infinity if no data points

def main(args=None):
    rclpy.init(args=args)
    node = LDSDistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
