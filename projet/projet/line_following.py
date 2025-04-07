import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge # convertit les images de ROS en OpenCV
import cv2 # utilisé pour le traitement d'image
import numpy as np

from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class LineFollowingNode(Node):
    def __init__(self):
        super().__init__('line_following_node') # initialisation du noeud avec le nom "line_following_node"

        # Choix du côté du rond-point
        self.declare_parameter('side','left')
        self.side = self.get_parameter('side').get_parameter_value().string_value
        self.declare_parameter('interface','/image_raw') #RAJOUTER /camera/image_raw/compressed si on veut interfacer
        self.interface = self.get_parameter('interface').get_parameter_value().string_value

        if self.interface == '/image_raw':
            self.image_subscriber = self.create_subscription(Image, self.interface, self.image_callback, 10) # création d'un suscriber qui écoute les images de la camera
        else:
            self.image_subscriber = self.create_subscription(CompressedImage, self.interface, self.image_callback, 10) # création d'un suscriber qui écoute les images de la camera
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10) # envoie les commandes de déplacement
        
        # instanciation pour convertir l'image de la camera en image OpenCV
        self.bridge = CvBridge()

        self.get_logger().info('Line following node started.')

        # Seuil HSV (Hue, Saturation, Value) pour détecter les couleurs (plus précisément pour isoler les pixels rouges et verts)
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 100, 100])
        self.upper_red2 = np.array([179, 255, 255])
        self.lower_green = np.array([40, 100, 100])
        self.upper_green = np.array([80, 255, 255])

        # définition des paramètres pour calculer le centre de la trajectoire avec un PID
        self.previous_error = 0.0 # Variable pour garder une trace de l'erreur dans le controle PID
        # utilisé pour ajuster la direction du robot
        self.last_known_center = 0.0 # dernière position centrale connue du robot

    # Fonction qui est appelée à chaque fois qu'une nouvelle image est reçue
    def image_callback(self, img_msg):
        # conversion de l'image en OpenCV
        if self.interface == '/image_raw':
            img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        else:
            np_arr = np.asarray(img_msg.data, dtype = np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            cv2.imshow('Camera View', img)
        
        height, width, _ = img.shape

        # définition de la "region of interest" du champ de vision de la camera
        # Pour éviter d'etre parasité par les autres obstacles qui sont formés de lignes rouges
        roi = img[height // 3:, :] # On ignore le huitième inférieur

        # Conversion de RGB en HSV (meilleurs pour la détection des couleurs)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Binarisation de l'image pour isoler les pixels rouges et verts
        mask_red1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask_red2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_green = cv2.inRange(hsv, self.lower_green, self.upper_green)
        
        # Morphologie (filtrage pour enlever le bruit des filtres)
        kernel = np.ones((5, 5), np.uint8)
        mask_red_clean = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
        mask_red_clean = cv2.morphologyEx(mask_red_clean, cv2.MORPH_CLOSE, kernel)
        mask_green_clean = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
        mask_green_clean = cv2.morphologyEx(mask_green_clean, cv2.MORPH_CLOSE, kernel)

        # Contours des limites droite et gauche du chemin à suivre
        contours_red, _ = cv2.findContours(mask_red_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_green, _ = cv2.findContours(mask_green_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # On ne garde que la plus grande aire formée par les contours (= correspond à la ligne)
        if contours_red:
            contours_red = [max(contours_red, key=cv2.contourArea)]
        if contours_green:
            contours_green = [max(contours_green, key=cv2.contourArea)]

        # Calcul de la trajectoire
        # Si les 2 lignes sont détectées
        if contours_red and contours_green:
            # Si des contours sont trouvés, on calcul les moments des contours
            M_red = cv2.moments(contours_red[0])
            M_green = cv2.moments(contours_green[0])
            # pour obtenir leur centre de gravité
            if M_red["m00"] != 0 and M_green["m00"] != 0:
                #calcul des coordonnées centrales des lignes rouge et verte, puis calcul du centre du chemin
                cx_red = int(M_red["m10"] / M_red["m00"])
                cy_red = int(M_red["m01"] / M_red["m00"])
                cx_green = int(M_green["m10"] / M_green["m00"])
                cy_green = int(M_green["m01"] / M_green["m00"])

                if cx_green < cx_red:
                    cx_center = (cx_red + cx_green) // 2
                    self.last_known_center = cx_center

                    error = cx_center - width // 2 #calcul de l'erreur : différence entre le centre du chemin et le centre de l'image 
                    # cette erreur est utilisée pour corriger la direction du robot
                    error_corrected = error  # pour compenser la tendance à gauche

                    #Application d'un PID simplifié (Proportionnel + dérivé) pour ajuster la direction du robot
                    k_p = 0.005
                    k_d = 0.005
                    derivative = error_corrected - self.previous_error
                    self.previous_error = error_corrected
                    
                    # Commandes de mouvement: le robot avance avec une vitesse linéaire et tourne selon l'erreur calculée (PID)
                    twist = Twist()

                    self.get_logger().info(f'1. cx_green = {cx_green}, cy_green = {cy_green}')
                    self.get_logger().info(f'1. cx_red = {cx_red}, cy_red = {cy_red}')

                    twist.linear.x = 0.1  # vitesse en ligne droite
                    if (cx_green > 10 and cy_green > 10) and (cx_red > 10 and cy_red > 10):
                        twist.angular.z = -k_p * error_corrected - k_d * derivative
                        self.get_logger().info(f'Ligne droite...')
                    else:
                        self.get_logger().info(f'Virage serré !')

                    cv2.circle(roi, (cx_green, cy_green), 5, (0, 255, 0), -1)  # cercle vert
                    cv2.circle(roi, (cx_red, cy_red), 5, (0, 0, 255), -1)  # cercle rouge

                    self.cmd_vel_publisher.publish(twist)

                else:
                    if cy_green > 90:
                        if self.side == 'left':
                            self.get_logger().info(f'Tourne à gauche au rond point')
                            twist = Twist()
                            twist.linear.x = 0.01
                            twist.angular.z = 0.3
                            self.cmd_vel_publisher.publish(twist)
                        elif self.side == 'right':
                            self.get_logger().info(f'Tourne à droite au rond point')
                            twist = Twist()
                            twist.linear.x = 0.01
                            twist.angular.z = -0.3
                            self.cmd_vel_publisher.publish(twist)
                        else : 
                            raise ValueError("Doit être un des côtés 'left' or 'right' !")

        elif contours_green and not contours_red:
            # Ligne verte seulement -> tourne vers la droite
            self.get_logger().info(f'Virage serré !')
            M_green = cv2.moments(contours_green[0])
            cx_green = int(M_green["m10"] / M_green["m00"])
            cy_green = int(M_green["m01"] / M_green["m00"])
            cv2.circle(roi, (cx_green, cy_green), 5, (0, 255, 0), -1)  # cercle vert
            self.get_logger().info(f'2. cx_green = {cx_green}, cy_green = {cy_green}')
            if M_green["m00"] != 0:
                if cy_green > 70:  # Quand trop près de la ligne
                    self.get_logger().info(f'Tourne vers la droite')
                    twist = Twist()
                    twist.linear.x = 0.01
                    twist.angular.z = -0.3
                    self.cmd_vel_publisher.publish(twist)

        elif contours_red and not contours_green:
            # Ligne rouge seulement -> tourne vers la gauche
            self.get_logger().info(f'Virage serré !')
            M_red = cv2.moments(contours_red[0])
            cx_red = int(M_red["m10"] / M_red["m00"])
            cy_red = int(M_red["m01"] / M_red["m00"])
            cv2.circle(roi, (cx_red, cy_red), 5, (0, 0, 255), -1)  # cercle rouge
            self.get_logger().info(f'2. cx_red = {cx_red}, cy_red = {cy_red}')
            if M_red["m00"] != 0:
                if cy_red > 90:    # Quand trop près de la ligne
                    self.get_logger().info(f'Tourne vers la gauche')
                    twist = Twist()
                    twist.linear.x = 0.01
                    twist.angular.z = 0.3
                    self.cmd_vel_publisher.publish(twist)

        else:
            # Si on perd les 2 lignes, tourner pour les retrouver
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.3  # tourne sur place
            self.cmd_vel_publisher.publish(twist)

        # Optionnel : affichage debug
        cv2.imshow('ROI', roi)
        cv2.imshow('Red Mask', mask_red_clean)
        cv2.imshow('Green Mask', mask_green_clean)
        cv2.waitKey(1)

    # Fonction pour enregistrer la position latérale simulant une trace GPS
    def save_position(self, center_x):
        """Simule l’enregistrement GPS (ici uniquement une coordonnée latérale)."""
        self.gps_trace.append(center_x)
        if len(self.gps_trace) > 1000:
            self.gps_trace.pop(0)

    # Fonction pour arreter le robot
    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
