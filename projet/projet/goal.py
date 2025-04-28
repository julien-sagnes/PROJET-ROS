import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class GoalScorer(Node):
    def __init__(self):
        super().__init__('goal_scorer')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()

        self.ball_detected = False
        self.ball_x = 0
        self.image_width = 0

    def image_callback(self, msg):
        # Convertir ROS Image en OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Traiter l'image
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Détecter la balle (ajuste ces valeurs selon la couleur de ta balle)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([40, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Trouver les contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            self.ball_detected = True
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M['m00'] > 0:
                self.ball_x = int(M['m10'] / M['m00'])
                self.image_width = cv_image.shape[1]
        else:
            self.ball_detected = False

        self.move_robot()

    def move_robot(self):
        twist = Twist()

        if self.ball_detected:
            center_x = self.image_width // 2
            error = self.ball_x - center_x

            # Contrôle proportionnel pour tourner vers la balle
            twist.angular.z = -0.002 * error
            twist.linear.x = 0.1
        else:
            # Si pas de balle détectée, tourner pour chercher
            twist.angular.z = 0.3
            twist.linear.x = 0.0

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = GoalScorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()