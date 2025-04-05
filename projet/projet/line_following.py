import rclpy
from rclpy.node import Node
import numpy  as np
from cv_bridge import *
import cv2

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class LineFollowing(Node):

    def __init__(self):
        super().__init__('line_following_node')

        self.declare_parameter('linear_scale',0.2)
        self.declare_parameter('side','red')  # Savoir si on doit prendre le rond point à gauche ('green') ou à droite ('red')

        self.linear_scale = self.get_parameter('linear_scale').get_parameter_value().double_value 
        self.side = self.get_parameter('side').get_parameter_value().string_value

        # Pour le traitement d'images par OpenCV
        self.bridge = CvBridge()

        # Abonné à l'image de la caméra
        self.subscriber = self.create_subscription(Image, '/image_raw', self.image_callback, 10)

        # Publie sur le topic qui contrôle le robot
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist = Twist()


    def image_callback(self, msg):
        # Convertir l'image ROS en image OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')        

        # Traitement de l'image
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        if self.side == 'green':
            lower_green = np.array([35, 100, 100])
            upper_green = np.array([85, 255, 255])
            mask = cv2.inRange(hsv, lower_green, upper_green)

        elif self.side == 'red':
            lower_red = np.array([0, 70, 50])
            upper_red = np.array([10, 255, 255])
            mask = cv2.inRange(hsv, lower_red, upper_red)
        
        else:
            self.get_logger().info(f"'side' : {self.side} doit être 'red' ou 'green'")
            # Shut everything down
            self.destroy_node()
            rclpy.shutdown()

        # Détection des contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Calcul des moments
            M = cv2.moments(mask)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])

                # Logique pour suivre la ligne rouge sur le côté droit
                self.follow_line(msg.step, cx)

        
    def follow_line(self, taille, cx):

        img_center = taille

        if self.side == 'green':
            target_position = img_center / 2  # Cible à gauche de l'image
            error = target_position - cx

        elif self.side == 'red':
            target_position = img_center + (img_center / 2)  # Cible à droite de l'image
            error = cx - target_position

        # Contrôle proportionnel simple
        kp = 0.01
        self.twist.angular.z = kp * error
        self.twist.linear.x = 0.2  # Vitesse constante

        self.publisher.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)

    # Init node
    line_following_node = LineFollowing()
    rclpy.spin(line_following_node)

    # Pour que les paramètres apparaissent avec la commande ros2 param list
    thread = threading.Thread(target = rclpy.spin, args = (line_following_node,), daemon=True)
    thread.start()


if __name__ == '__main__':
    main()

