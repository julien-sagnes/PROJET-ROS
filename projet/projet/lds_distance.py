import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class GoalBall(Node):
    def __init__(self):
        super().__init__('goal_ball')

        # Paramètre pour l'interface caméra
        self.declare_parameter('interface', '/image_raw')
        self.interface = self.get_parameter('interface').get_parameter_value().string_value

        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_sub = self.create_subscription(Image, self.interface, self.image_callback, 10)
        self.lds_sub = self.create_subscription(Float32MultiArray, '/lds_distances', self.lds_callback, 10)
        
        self.bridge = CvBridge()

        # Variables internes
        self.ball_detected = False
        self.ball_x = 0
        self.image_width = 0
        self.front_distance = float('inf')  # distance devant le robot
        self.goal_detected = False

        self.get_logger().info("Noeud GoalBall lancé 🚀")

    def image_callback(self, msg):
        # Traitement image OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Détecter balle jaune
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([40, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        cv2.imshow('Camera', cv_image)
        cv2.imshow('Mask Ball', mask)
        cv2.waitKey(1)

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

    def lds_callback(self, msg):
        # msg.data = [front_mean, left_mean, right_mean, back_mean]
        self.front_distance = msg.data[0]

        # Si la distance avant est grande (poteaux détectés → ouverture)
        if self.front_distance > 1.2:  # À ajuster selon ton terrain
            self.goal_detected = True
            self.get_logger().info("Ouverture détectée : But en vue 🥅")
        else:
            self.goal_detected = False

    def move_robot(self):
        twist = Twist()

        if self.ball_detected:
            center_x = self.image_width // 2
            error = self.ball_x - center_x

            # Si le but est détecté (balle alignée + but libre), pousser rapidement
            if self.goal_detected:
                twist.linear.x = 0.3  # Avancer vite pour marquer
                twist.angular.z = -0.002 * error
            else:
                twist.linear.x = 0.1  # Suivi lent normal
                twist.angular.z = -0.002 * error
        else:
            twist.angular.z = 0.3  # Cherche la balle
            twist.linear.x = 0.0

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
