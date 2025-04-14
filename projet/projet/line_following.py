import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge # convertit les images de ROS en OpenCV
import cv2 # utilisé pour le traitement d'image
import numpy as np
import click
import threading
import matplotlib.pyplot as plt 

from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class LineFollowingNode(Node):
    def __init__(self):
        super().__init__('line_following_node') # initialisation du noeud avec le nom "line_following_node"

        # Choix de la vitesse global
        self.declare_parameter('linear_scale',0.07)
        self.linear_scale = self.get_parameter('linear_scale').get_parameter_value().double_value
        # Choix du côté du rond-point
        self.declare_parameter('side','right')
        self.side = self.get_parameter('side').get_parameter_value().string_value
        # Choix entre simulation et réel
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
        self.lower_red1 = np.array([0, 60, 50])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 50, 50])
        self.upper_red2 = np.array([180, 255, 255])

        # Vert plus large
        self.lower_green = np.array([25, 30, 30])
        self.upper_green = np.array([95, 255, 255])

        # définition des paramètres pour calculer le centre de la trajectoire avec un PID
        self.previous_error = 0.0 # Variable pour garder une trace de l'erreur dans le controle PID
        # utilisé pour ajuster la direction du robot
        self.last_known_center = 0.0 # dernière position centrale connue du robot

        # Paramètre à changer pour le passage du rond point à droite (True) ou à gauche (False)
        self.passage_a_droite = False

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
        roi = img[height // 4:, :] # On ignore le quart inférieur

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
                cx_center = (cx_green+cx_red)//2
                cy_center = (cy_green + cy_red)//2

                self.get_logger().info(f'1. cx_green = {cx_green}, cy_green = {cy_green}')
                self.get_logger().info(f'1. cx_red = {cx_red}, cy_red = {cy_red}')
                cv2.circle(roi, (cx_green, cy_green), 5, (0, 255, 0), -1)  # cercle vert
                cv2.circle(roi, (cx_red, cy_red), 5, (0, 0, 255), -1)  # cercle rouge 
                cv2.circle(roi, (cx_center, cy_center), 5, (255, 0, 0), -1) # cercle bleu
                print (f'cx_red = {cx_red} , cy_red = {cy_red}')
                print (f'cx_green = {cx_green} , cy_green = {cy_green}')
                print (f'center_x = {cx_center} , center_y = {cy_center}')
                """
                Idée pour gérer la rotation du rond point (à faire) :
                if cx_green < cx_red and cy_red > 200:  # Si on est proche de la rouge
                    cx_center = (cx_red - 10 + cx_green) // 2   # On va un peu plus vers la gauche
                    self.last_known_center = cx_center

                    error = cx_center - width // 2 #calcul de l'erreur : différence entre le centre du chemin et le centre de l'image 
                    # cette erreur est utilisée pour corriger la direction du robot
                    error_corrected = error  # pour compenser la tendance à gauche

                    #Application d'un PID simplifié (Proportionnel + dérivé) pour ajuster la direction du robot
                    k_p = 0.01
                    k_d = 0.01
                    derivative = error_corrected - self.previous_error
                    self.previous_error = error_corrected
                    
                    # Commandes de mouvement: le robot avance avec une vitesse linéaire et tourne selon l'erreur calculée (PID)
                    twist = Twist()
                    twist.linear.x = self.linear_scale
                    twist.angular.z = -k_p * error_corrected - k_d * derivative

                    self.get_logger().info(f'Virage !')

                    self.cmd_vel_publisher.publish(twist)

                elif cx_green < cx_red and cy_green > 200:  # Si on est proche de la verte
                    cx_center = (cx_red + 10 + cx_green) // 2   # On va un peu plus vers la droite
                    self.last_known_center = cx_center

                    error = cx_center - width // 2 #calcul de l'erreur : différence entre le centre du chemin et le centre de l'image 
                    # cette erreur est utilisée pour corriger la direction du robot
                    error_corrected = error  # pour compenser la tendance à gauche

                    #Application d'un PID simplifié (Proportionnel + dérivé) pour ajuster la direction du robot
                    k_p = 0.01
                    k_d = 0.01
                    derivative = error_corrected - self.previous_error
                    self.previous_error = error_corrected
                    
                    # Commandes de mouvement: le robot avance avec une vitesse linéaire et tourne selon l'erreur calculée (PID)
                    twist = Twist()
                    twist.linear.x = self.linear_scale
                    twist.angular.z = -k_p * error_corrected - k_d * derivative

                    self.get_logger().info(f'Virage !')

                    self.cmd_vel_publisher.publish(twist)
                """
                if cx_green < cx_red :
                    

                    if cx_center < 320 and cy_center < 150 and cy_green > 300 : # virage à gauche

                        self.get_logger().info(f'virage à gauche en approche !')
                        # Commandes de mouvement: le robot avance avec une vitesse linéaire et tourne selon l'erreur calculée (PID)
                        twist = Twist()
                        twist.linear.x = 0.03 # on maintient une vitesse réduite
                        twist.angular.z = 0.5
                        self.cmd_vel_publisher.publish(twist)

                    elif cx_red > 480 and cy_red > 255 and cx_green < 160 and cy_green < 85 :
                        self.get_logger().info(f'virage à droite !!')
                        twist = Twist()
                        twist.linear.x = 0.01 # on maintient une vitesse réduite
                        twist.angular.z = 0.5
                        self.cmd_vel_publisher.publish(twist)


                    elif cx_center >0 and cy_center < 170 and cx_red > 320 and cy_red < 300 and cx_green < 150 and cy_green < 110 :
                        self.get_logger().info(f'Je vois du vert en haut à gauche')
                        twist = Twist()
                        twist.linear.x = 0.05 # on maintient une vitesse réduite
                        twist.angular.z = 0.5
                        self.cmd_vel_publisher.publish(twist)

                    elif cx_center >0 and cy_center < 170 and cx_red > 320 and cy_red < 300 and cx_green > 150 and cy_green > 110 :
                        self.get_logger().info(f'On arrive au rond point')
                        twist = Twist()
                        twist.linear.x = 0.05 # on maintient une vitesse réduite
                        twist.angular.z = -0.9
                        self.cmd_vel_publisher.publish(twist)
                    
                    else :
                        self.get_logger().info("on continue tout droit je suis dans le ELSE avec 2 couleurs!!")
                        twist = Twist()
                        twist.linear.x = self.linear_scale # on maintient la vitesse en ligne droite
                        twist.angular.z = 0.0
                        self.cmd_vel_publisher.publish(twist)

                if cx_red < cx_green :
                    self.get_logger().info("On est au rond-point, je vais où ??")
                    if cx_green > 500 and cy_green >160 :
                        if self.passage_a_droite == True :
                            self.get_logger().info("JE PASSE A DROITE !!!")
                            twist = Twist()
                            twist.linear.x = 0.02 # on maintient la vitesse en ligne droite
                            twist.angular.z = -0.5
                            self.cmd_vel_publisher.publish(twist)
                        
                        else :
                            self.get_logger().info("JE PASSE A GAUCHE !!!")
                            twist = Twist()
                            twist.linear.x = 0.02 # on maintient la vitesse en ligne droite
                            twist.angular.z = 0.5
                            self.cmd_vel_publisher.publish(twist)

                



        elif contours_green and not contours_red:
            # Ligne verte seulement -> tourne vers la droite
            self.get_logger().info(f'Virage à droite à venir VERT ONLYY!')
            M_green = cv2.moments(contours_green[0])
            
            if M_green["m00"] != 0:
                cx_green = int(M_green["m10"] / M_green["m00"])
                cy_green = int(M_green["m01"] / M_green["m00"])
                cv2.circle(roi, (cx_green, cy_green), 5, (0, 255, 0), -1)  # cercle vert
                self.get_logger().info(f'2. cx_green = {cx_green}, cy_green = {cy_green}')

                if cx_green > 100 and cy_green > 170 :  # Quand trop près de la ligne
                    self.get_logger().info(f'Tourne vers la droite VERT ONLY')
                    twist = Twist()
                    twist.linear.x = 0.03
                    twist.angular.z = -0.55
                    self.cmd_vel_publisher.publish(twist)

                

            else :
                    self.get_logger().info("on continue quand meme VERT ONLY!!")
                    twist = Twist()
                    twist.linear.x = self.linear_scale# on maintient la vitesse en ligne droite
                    twist.angular.z = 0.0
                    self.cmd_vel_publisher.publish(twist)

        elif contours_red and not contours_green:
            # Ligne rouge seulement -> tourne vers la gauche
            self.get_logger().info(f'Virage serré à gauche! ROUGE ONLY!!!')
            M_red = cv2.moments(contours_red[0])
            
            #while cx_red 
            
            if M_red["m00"] != 0 :
                cx_red = int(M_red["m10"] / M_red["m00"])
                cy_red = int(M_red["m01"] / M_red["m00"])
                cv2.circle(roi, (cx_red, cy_red), 5, (0, 0, 255), -1)  # cercle rouge
                self.get_logger().info(f'2. cx_red = {cx_red}, cy_red = {cy_red}')

                if cx_red < 400 and cy_red < 300 :
                    self.get_logger().info(f'Il faut tourner à gauche ROUGE ONLY!!')
                    twist = Twist()
                    twist.linear.x = 0.02
                    twist.angular.z = 0.2
                    self.cmd_vel_publisher.publish(twist)
            
            else :
                    self.get_logger().info("on continue quand meme ROUGE ONLY!!")
                    twist = Twist()
                    twist.linear.x = self.linear_scale# on maintient la vitesse en ligne droite
                    twist.angular.z = 0.0
                    self.cmd_vel_publisher.publish(twist)

        else:
            # Si on perd les 2 lignes, tourner pour les retrouver
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.3  # tourne sur place
            self.cmd_vel_publisher.publish(twist)

        # Optionnel : affichage debug
        
        #cv2.imshow('Red Mask', mask_red_clean)
        #cv2.imshow('Green Mask', mask_green_clean)
        cv2.imshow('ROI', roi)
        cv2.imshow('Red Mask (raw)', mask_red)
        cv2.imshow('Green Mask (raw)', mask_green)
        cv2.waitKey(1)
        

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
