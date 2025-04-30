import rclpy
import atexit
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineFollowerWithObstacle(Node):
    def __init__(self):
        super().__init__('line_follower_with_obstacle')

        self.declare_parameter('linear_speed', 0.1)
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.state = 'FOLLOW_LINE'
        self.avoid_direction = None
        self.avoid_start_time = None
        self.return_start_time = None

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        height, width, _ = image.shape
        roi = hsv[height//2:, :]  # Region of interest

        # Masques de couleurs
        # Masque rouge (deux plages)
        lower_red1 = (0, 100, 100)
        upper_red1 = (10, 255, 255)
        lower_red2 = (160, 100, 100)
        upper_red2 = (180, 255, 255)
        red_mask = cv2.inRange(roi, lower_red1, upper_red1) | cv2.inRange(roi, lower_red2, upper_red2)

        # Masque vert
        green_mask = cv2.inRange(roi, (35, 50, 50), (85, 255, 255))

        # Masque bleu
        blue_mask = cv2.inRange(roi, (90, 50, 50), (130, 255, 255))

        cv2.imshow("Red mask", red_mask)
        cv2.imshow("Green mask", green_mask)
        cv2.imshow("Blue mask", blue_mask)
        cv2.waitKey(1)
        atexit.register(cv2.destroyAllWindows)

        # Moments des lignes
        cx_red = self.get_centroid(red_mask)
        cx_green = self.get_centroid(green_mask)

        # Détection d'obstacle via moment du masque bleu
        M_blue = cv2.moments(blue_mask)
        obstacle_detected = M_blue["m00"] > 1000  # seuil à ajuster
        cx_blue = int(M_blue["m10"] / M_blue["m00"]) if obstacle_detected else None

        twist = Twist()

        # FSM
        if self.state == 'FOLLOW_LINE':
            if obstacle_detected:
                self.state = 'AVOID_OBSTACLE'
                self.avoid_direction = 'left' if cx_blue > width // 2 else 'right'
                self.avoid_start_time = self.get_clock().now()
                self.get_logger().info(f"Obstacle detected, avoiding to the {self.avoid_direction}")

            elif cx_red is not None and cx_green is not None:
                self.get_logger().info(f"Line following...")  
                center = (cx_red + cx_green) // 2
                error = center - width // 2
                twist.linear.x = self.linear_speed
                twist.angular.z = -error / 100.0
            elif cx_green is not None:  # Si on ne voit que la ligne verte
                self.get_logger().info(f"Green only : to the right...")  
                twist.linear.x = self.linear_speed
                twist.angular.z = -0.5  # Tourner à gauche pour rattraper la rouge
            elif cx_red is not None:  # Si on ne voit que la ligne rouge
                self.get_logger().info(f"Red only : to the left...")  
                twist.linear.x = self.linear_speed
                twist.angular.z = 0.5  # Tourner à droite pour rattraper la verte

            else:
                self.get_logger().info(f"Line lost : STOP") 
                twist.linear.x = 0.0
                twist.angular.z = 0.0  # fail-safe

        elif self.state == 'AVOID_OBSTACLE':
            elapsed = (self.get_clock().now() - self.avoid_start_time).nanoseconds / 1e9

            if elapsed < 10.0:
                # Phase 1: déviation dans le sens opposé pour s'éloigner un peu
                twist.linear.x = self.linear_speed * 0.7
                twist.angular.z = 0.5 if self.avoid_direction == 'left' else -0.5
            elif elapsed < 20.0:
                # Phase 2: contournement dans la bonne direction
                twist.linear.x = self.linear_speed * 0.7
                twist.angular.z = -0.5 if self.avoid_direction == 'left' else 0.5
            else:
                self.state = 'FOLLOW_LINE'
                self.return_start_time = self.get_clock().now()
                self.get_logger().info("Finished avoiding obstacle, returning to path")

        self.publisher.publish(twist)

    def get_centroid(self, mask):
        M = cv2.moments(mask)
        if M["m00"] > 0:
            return int(M["m10"] / M["m00"])
        return None


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerWithObstacle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
