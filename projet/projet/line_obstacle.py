import rclpy
import atexit
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class LineFollowerWithObstacle(Node):
    def __init__(self):
        super().__init__('line_obstacle_node')

        #Déclaration des paramètres
        self.declare_parameter('linear_speed', 0.03)
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.declare_parameter('angular_speed', 0.4)
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
        self.declare_parameter('distance_limit', 0.35)
        self.distance_limit = self.get_parameter('distance_limit').get_parameter_value().double_value
        self.declare_parameter('interface','/image_raw') #RAJOUTER /camera/image_raw/compressed si on veut interfacer
        self.interface = self.get_parameter('interface').get_parameter_value().string_value

        # Déclaration des subscriber
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        """
        if self.interface == '/image_raw':
            self.image_subscriber = self.create_subscription(Image, self.interface, self.image_callback, 10) # création d'un suscriber qui écoute les images de la camera
        else:
            self.image_subscriber = self.create_subscription(CompressedImage, self.interface, self.image_callback, 10) # création d'un suscriber qui écoute les images de la camera
        """

        # Déclaration des publisher
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Déclaration d'attributs
        self.bridge = CvBridge()
        self.state = 'FOLLOW_LINE'
        self.avoid_direction = None
        self.avoid_start_time = None
        self.return_start_time = None

    # méthode qui utilise le retour d'images
    def image_callback(self, msg):
        if self.interface == '/image_raw':
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        else:
            np_arr = np.asarray(msg.data, dtype = np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            cv2.imshow('Camera View', img)

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        height, width, _ = img.shape
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
        #self.get_logger().info(f' M_blue = {M_blue}')
        obstacle_detected = M_blue["m00"] > 500  # seuil à ajuster
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
                if error < 0 :
                    twist.linear.x = self.linear_speed
                    twist.angular.z = error / 150.0

                elif error > 0 :
                    twist.linear.x = self.linear_speed
                    twist.angular.z = -error / 150.0
                
                else :
                    twist.linear.x = self.linear_speed
                    twist.angular.z = 0.0

            elif cx_green is not None:  # Si on ne voit que la ligne verte
                self.get_logger().info(f"Green only : to the right...")  
                twist.linear.x = self.linear_speed
                twist.angular.z = -self.angular_speed  # Tourner à gauche pour rattraper la rouge
            elif cx_red is not None:  # Si on ne voit que la ligne rouge
                self.get_logger().info(f"Red only : to the left...")  
                twist.linear.x = self.linear_speed
                twist.angular.z = self.angular_speed  # Tourner à droite pour rattraper la verte

            else:
                self.get_logger().info(f"Line lost : STOP") 
                twist.linear.x = 0.0
                twist.angular.z = 0.1 # fail-safe

        elif self.state == 'AVOID_OBSTACLE':
            elapsed = (self.get_clock().now() - self.avoid_start_time).nanoseconds / 1e9

            if elapsed < 2 :
                self.get_logger().info(f'time elapsed < 2 = {elapsed}')
                # Phase 1: déviation dans le sens opposé pour s'éloigner un peu
                twist.linear.x = self.linear_speed
                twist.angular.z = self.angular_speed if self.avoid_direction == 'left' else -self.angular_speed

            #elif elapsed < 4.0:
                #self.get_logger().info(f'time elapsed < 4 = {elapsed}')
                # Phase 2: contournement dans la bonne direction
                #twist.linear.x = self.linear_speed 
                #twist.angular.z = -self.angular_speed if self.avoid_direction == 'left' else self.angular_speed
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

    def lidar_callback(self, msg) :
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
            cmd.linear.x = 0.02
            cmd.angular.z = 0.0 
            self.publisher.publish(cmd)
            self.get_logger().info("Tout Droit")


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerWithObstacle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
