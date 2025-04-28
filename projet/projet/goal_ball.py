import rclpy
from rclpy.node import Node
import time
from cv_bridge import CvBridge
import cv2
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class GoalBall(Node):
    def __init__(self):
        super().__init__('goal_ball')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)

        self.bridge = CvBridge()

        # Etats possibles du challenge
        self.state = 'SEARCH_BALL'

        # Variables internes
        self.ball_detected = False
        self.ball_x = 0

        self.pushing = False
        self.start_time = None
        self.push_duration = 3.5  # Durée de la poussée en secondes

        self.image_width = None
        self.image_height = None

        self.get_logger().info("Noeud GoalBall lancé")

    def image_callback(self, msg):
        # Lire dimensions de l'image directement depuis msg
        self.image_width = msg.width
        self.image_height = msg.height

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([40, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            self.ball_detected = True
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M['m00'] > 0:
                self.ball_x = int(M['m10'] / M['m00'])
                self.ball_y = int(M['m01'] / M['m00'])
        else:
            self.ball_detected = False

        # Affichage pour debug
        cv2.imshow("Image", cv_image)
        cv2.imshow("Mask jaune", mask)
        cv2.waitKey(1)

        self.update_state()

    def update_state(self):
        twist = Twist()

        if self.state == 'SEARCH_BALL':
            if self.ball_detected:
                # Vérifie que la balle est assez basse dans l'image (donc proche)
                ball_close_enough = self.ball_y > (0.75 * self.image_height)  # 75% de la hauteur de l'image
                if ball_close_enough:
                    self.get_logger().info("Balle très proche, début de la poussée")
                    self.state = 'PUSH_BALL'
                    self.start_time = time.time()
                else:
                    # Sinon, avancer vers la balle
                    center_offset = (self.ball_x - self.image_width // 2) / (self.image_width // 2)  # Normalisé [-1,1]
                    twist.linear.x = 0.15
                    twist.angular.z = -0.3 * center_offset  # Diriger légèrement vers la balle

        elif self.state == 'PUSH_BALL':
            elapsed = time.time() - self.start_time
            if elapsed < self.push_duration:
                twist.linear.x = 0.15
                twist.angular.z = 0.2  # Pousser vers la gauche
            else:
                self.get_logger().info("Poussée terminée")
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.state = 'SEARCH_BALL'  # Revenir à la recherche

        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = GoalBall()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
