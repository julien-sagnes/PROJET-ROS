import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineObstacle(Node):

    def __init__(self):
        super().__init__('line_obstacle')

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()

        self.state = 'FOLLOW_LINE'  # Autres états : AVOID_OBSTACLE, RETURN
        self.avoid_direction = None

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        height, width, _ = image.shape
        roi = hsv[height//2:, :]

        # Masques couleur
        red_mask = cv2.inRange(roi, (0, 70, 50), (10, 255, 255))
        green_mask = cv2.inRange(roi, (40, 70, 50), (80, 255, 255))
        blue_mask = cv2.inRange(roi, (90, 50, 50), (130, 255, 255))

        # Centres lignes
        def get_centroid(mask):
            M = cv2.moments(mask)
            if M["m00"] > 0:
                return int(M["m10"] / M["m00"])
            return None

        cx_red = get_centroid(red_mask)
        cx_green = get_centroid(green_mask)
        cx_blue = get_centroid(blue_mask)

        twist = Twist()

        # Obstacle détecté ?
        obstacle_detected = cv2.countNonZero(blue_mask) > 5000  # seuil ajustable

        if obstacle_detected and self.state == 'FOLLOW_LINE':
            self.state = 'AVOID_OBSTACLE'
            self.avoid_direction = 'left' if cx_blue > width // 2 else 'right'
            self.get_logger().info(f"Obstacle detected, avoiding to the {self.avoid_direction}")

        # FSM
        if self.state == 'FOLLOW_LINE':
            if cx_red and cx_green:
                center = (cx_red + cx_green) // 2
                error = center - width // 2
                twist.linear.x = 0.1
                twist.angular.z = -error / 100.0
        elif self.state == 'AVOID_OBSTACLE':
            # On force une déviation
            error_offset = 100 if self.avoid_direction == 'left' else -100
            center = (cx_red + cx_green) // 2 if (cx_red and cx_green) else width // 2
            error = (center + error_offset) - width // 2
            twist.linear.x = 0.08
            twist.angular.z = -error / 80.0

            # L'obstacle a disparu ?
            if cv2.countNonZero(blue_mask) < 1000:
                self.state = 'RETURN'
        elif self.state == 'RETURN':
            if cx_red and cx_green:
                center = (cx_red + cx_green) // 2
                error = center - width // 2
                twist.linear.x = 0.1
                twist.angular.z = -error / 50.0

                # Retour stabilisé ?
                if abs(error) < 10:
                    self.state = 'FOLLOW_LINE'

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = LineObstacleFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()