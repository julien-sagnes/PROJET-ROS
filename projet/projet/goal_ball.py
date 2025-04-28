import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

import time

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
        self.contouring_ball = False  # Flag pour savoir si le robot doit contourner la balle
        self.pushing_ball = False  # Flag pour savoir si le robot est en train de pousser la balle
        self.start_time = None  # Moment où le robot commence à pousser la balle

        self.get_logger().info("Noeud GoalBall lancé")

    def image_callback(self, msg):
        # Traitement image OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Détecter balle jaune
        mask1 = cv2.inRange(hsv, np.array([20, 100, 100]), np.array([40, 255, 255]))  # Jaune clair
        mask2 = cv2.inRange(hsv, np.array([15, 40, 40]), np.array([45, 255, 150]))    # Jaune foncé
        mask = cv2.bitwise_or(mask1, mask2)

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

        # Si la distance avant est grande (poteaux détectés)
        if self.front_distance > 1.2:  # À ajuster selon ton terrain
            self.goal_detected = True
            self.get_logger().info("But en vue")
        else:
            self.goal_detected = False

    def move_robot(self):
        twist = Twist()

        # Si la balle est détectée
        if self.ball_detected:
            center_x = self.image_width // 2
            error = self.ball_x - center_x

            # Si on commence à pousser la balle pendant 2 secondes vers la gauche
            if not self.pushing_ball:
                self.pushing_ball = True
                self.start_time = time.time()

            # Si le robot pousse la balle vers la gauche
            if self.pushing_ball:
                twist.linear.x = 0.2  # Vitesse modérée pour pousser la balle
                twist.angular.z = 0.1  # Légère rotation à droite pour pousser la balle vers la gauche
                # Vérifier si 2 secondes se sont écoulées
                if time.time() - self.start_time > 2:
                    self.pushing_ball = False  # Arrêter de pousser la balle après 2 secondes
                    self.contouring_ball = True  # Passer à l'étape suivante : contourner la balle
                    self.get_logger().info("Balle poussée vers la gauche, contournement imminent")
            
            # Si le robot doit contourner la balle (se positionner à sa gauche)
            if self.contouring_ball:
                # Rotation à gauche autour de la balle
                twist.linear.x = 0.0
                twist.angular.z = 0.3  # Vitesse de rotation à ajuster si nécessaire
                # Laisser tourner pendant un temps défini (environ 1-2 secondes)
                if time.time() - self.start_time > 4:  # 4 secondes pour contourner (ajuster si nécessaire)
                    self.contouring_ball = False  # Arrêter le contournement
                    self.get_logger().info("Robot contourné la balle, prêt à pousser vers le but")
                    # Maintenant, le robot est bien placé pour pousser la balle vers le but
                    twist.angular.z = 0.0  # Arrêter de tourner

        # Publier les commandes de mouvement
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
