import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np
import cv2
import math
import time

class GoalBall(Node):
    def __init__(self):
        super().__init__('goal_ball')

        # Paramètres
        self.declare_parameter('rotate_direction', 'left') # Si le but est davantage à droite ou à gauche (inversé)
        self.declare_parameter('linear_speed', 0.05)
        self.declare_parameter('push_duration', 0.0)
        self.declare_parameter('ball_reposition_distance', 0.7)
        self.declare_parameter('rotate_speed', 0.3)

        self.rotate_direction = self.get_parameter('rotate_direction').get_parameter_value().string_value
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.push_duration = self.get_parameter('push_duration').get_parameter_value().double_value
        self.ball_reposition_distance = self.get_parameter('ball_reposition_distance').get_parameter_value().double_value
        self.rotate_speed = self.get_parameter('rotate_speed').get_parameter_value().double_value

        # Choix entre simulation et réel
        self.declare_parameter('interface','/image_raw') #RAJOUTER /camera/image_raw/compressed si on veut interfacer
        self.interface = self.get_parameter('interface').get_parameter_value().string_value

        # ROS
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        if self.interface == '/image_raw':
            self.image_subscriber = self.create_subscription(Image, self.interface, self.image_callback, 10) # création d'un suscriber qui écoute les images de la camera
        else:
            self.image_subscriber = self.create_subscription(CompressedImage, self.interface, self.image_callback, 10) # création d'un suscriber qui écoute les images de la camera

        self.bridge = CvBridge()

        # État interne
        self.ball_detected = False
        self.ball_x = 0
        self.ball_y = 0
        self.image_width = 0
        self.image_height = 0

        self.lidar_ranges = []

        self.state = 'SEARCH_BALL'
        self.start_timer = time.time()
        self.wait_time = 6.0
        self.turning_phase = None
        self.push_start_time = None
        self.rotation_start_time = None
        self.orbit_start_time = None
        self.orbit_duration = 3.0
        self.lost_goal_start_time = None

        # 90°
        self.rotation_duration = (math.pi / 2) / self.rotate_speed  # 90° rotation duration

        self.get_logger().info("Node GoalBall started")

    def image_callback(self, msg):
        if self.interface == '/image_raw':
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        else:
            np_arr = np.asarray(msg.data, dtype = np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            cv2.imshow('Camera View', img)

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        self.image_height, self.image_width, _ = img.shape
        

        # Yellow mask for the ball
        lower_yellow = np.array([22, 80, 80])
        upper_yellow = np.array([32, 255, 255])

        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Nettoyage du masque : fermeture puis ouverture
        kernel = np.ones((5, 5), np.uint8)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)  # comble les trous
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)   # enlève le bruit

        # Optionnel : léger flou après le nettoyage
        yellow_mask = cv2.GaussianBlur(yellow_mask, (5, 5), 0)


        # Red mask for goal posts
        lower_red1 = np.array([0, 60, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 50, 50])
        upper_red2 = np.array([180, 255, 255])
        red_mask = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)
        # Masking lower part of the image to ignore red lines on the floor
        height_mask = int(self.image_height * 0.6)  # Ajuster selon besoin, ici on garde seulement le haut 60%
        red_mask[:height_mask, :] = red_mask[:height_mask, :]  # Conserve le haut
        red_mask[height_mask:, :] = 0  # Supprime le bas (où il y a des lignes rouges au sol)

        cv2.imshow("Yellow Mask", yellow_mask)
        cv2.imshow("Red Mask", red_mask)
        cv2.waitKey(1)

        # Detect ball
        contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M['m00'] > 0:
                self.ball_detected = True
                self.ball_x = int(M['m10'] / M['m00'])
                self.ball_y = int(M['m01'] / M['m00'])
            else:
                self.ball_detected = False
        else:
            self.ball_detected = False

        # Detect red objects (goal posts)
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filtrer les contours rouges les plus significatifs
        filtered_contours = [c for c in contours if cv2.contourArea(c) > 100]

        # Extraire les centres pour self.red_objects + les bounding boxes pour goal_x
        bounding_boxes = []

        for c in filtered_contours:
            M = cv2.moments(c)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                bounding_boxes.append(cv2.boundingRect(c))

        if len(bounding_boxes) >= 2:
            bounding_boxes.sort(key=lambda b: b[0])
            leftmost = bounding_boxes[0]
            rightmost = bounding_boxes[-1]

            self.goal_left_edge = leftmost[0]
            self.goal_right_edge = rightmost[0] + rightmost[2]

            self.goal_x = (self.goal_left_edge + self.goal_right_edge) // 2
        else:
            self.goal_x = None
            self.goal_left_edge = None
            self.goal_right_edge = None

        if self.goal_x is not None:
            # Affiche un cercle au centre du but
            cv2.circle(img, (self.goal_x, self.image_height // 3), 10, (0, 0, 255), -1) 
            # Affiche une ligne verticale à goal_x
            cv2.line(img, (self.goal_x, 0), (self.goal_x, self.image_height), (0, 0, 255), 2)
            # Affiche un cercle au centre de la balle
            cv2.circle(img, (self.ball_x, self.image_height // 3), 10, (0, 255, 255), -1) 
            # Affiche une ligne verticale à ball_x
            cv2.line(img, (self.ball_x, 0), (self.ball_x, self.image_height), (0, 255, 255), 2)  
            # Affiche une ligne verticale au centre de l'image pour référence
            cv2.line(img, (self.image_width // 2, 0), (self.image_width // 2, self.image_height), (0, 0, 0), 2)  # bleu

        cv2.imshow("Goal Visualization",img)

    def goal_centered(self):
        if self.goal_x is not None:
            center_x = self.image_width // 2

            # Si les deux poteaux sont détectés, on ajuste dynamiquement la tolérance
            if self.goal_left_edge is not None and self.goal_right_edge is not None:
                goal_width_px = abs(self.goal_right_edge - self.goal_left_edge)
                relative_width = goal_width_px / self.image_width  # entre 0.0 et 1.0

                # La tolérance est plus faible quand le but paraît étroit
                # Exemple : pour un but très large → tolérance jusqu'à 15%, très étroit → tolérance jusqu'à 5%
                tolerance = self.image_width * (0.25 + 0.5 * relative_width)
            else:
                # Cas sans info : tolérance classique
                tolerance = self.image_width * 0.30

            self.get_logger().warn(f"diff_goal = {abs(self.goal_x - center_x)}")
            self.get_logger().warn(f"tolerance_goal = {tolerance}")
            return abs(self.goal_x - center_x) < tolerance
        return False

    def ball_centered(self):
        if not self.ball_detected:
            return False
        center_x = self.image_width // 2

        return abs(self.ball_x - center_x) < self.image_width * 0.1 # 20% de tolérance

    def control_loop(self):
        twist = Twist()
        now = time.time()

        # Attente que la caméra se lance
        if now - self.start_timer < self.wait_time:
            self.get_logger().info("Temps de lancement...")
            self.get_logger().info(f"Attente durant {now - self.start_timer} / {self.wait_time}.")
            return

        if self.state == 'SEARCH_BALL':
            if self.ball_detected:
                if self.ball_y > self.image_height * self.ball_reposition_distance:
                    self.get_logger().warn("Balle proche, préparation à la poussée.")
                    self.push_start_time = now
                    self.state = 'PUSH_BALL'
                else:
                    twist.linear.x = self.linear_speed
                    center_x = self.image_width // 2
                    offset_x = self.ball_x - center_x
                    twist.angular.z = -0.001 * offset_x
                    self.get_logger().info("Balle détéctée, on s'en approche.")
            else:
                self.get_logger().info("Recherche de la balle")
                twist.angular.z = self.rotate_speed

        elif self.state == 'PUSH_BALL':
            if now - self.push_start_time < self.push_duration:
                twist.linear.x = self.linear_speed
                twist.angular.z = 0.2 if self.rotate_direction == 'left' else -0.2
            if self.goal_centered() and self.ball_centered():
                self.get_logger().warn("Alignement balle-but déjà parfait. On passe au tir !")
                self.state = 'SHOOT'
                self.push_start_time = time.time()
            else:
                self.get_logger().warn("Poussée terminée, passage à TURNING_BALL.")
                self.state = 'TURNING_BALL'
                self.turning_phase = 'REPOSITION_FRONT'

        elif self.state == 'TURNING_BALL':
            if self.turning_phase == 'REPOSITION_FRONT':
                # Si trop proche : on recule
                if self.ball_detected and self.ball_y > self.image_height * self.ball_reposition_distance:
                    twist.linear.x = - self.linear_speed
                    center_x = self.image_width // 2
                    offset_x = self.ball_x - center_x
                    twist.angular.z = -0.002 * offset_x
                else:
                    self.get_logger().info("Repositionnement terminé. Rotation initiale.")
                    self.turning_phase = 'ROTATE_90'
                    self.rotation_start_time = now

            elif self.turning_phase == 'ROTATE_90':
                if now - self.rotation_start_time < self.rotation_duration:
                    twist.angular.z = self.rotate_speed if self.rotate_direction == 'left' else -self.rotate_speed
                else:
                    self.get_logger().info("Fin de la rotation. Passage à l'orbite")
                    self.turning_phase = 'ORBIT'
                    self.orbit_start_time = now

            elif self.turning_phase == 'ORBIT':
                if now - self.orbit_start_time < self.orbit_duration:
                    self.get_logger().info(f"Orbite durant {now - self.orbit_start_time} / {self.orbit_duration}.")
                    twist.linear.x = self.linear_speed
                else:
                    self.get_logger().info("Pause orbite, on regarde la balle.")
                    self.turning_phase = 'CHECK_ALIGNMENT'
                    self.rotation_start_time = now

            elif self.turning_phase == 'CHECK_ALIGNMENT':
                # 1) Phase de rotation pour centrer la balle
                if not self.ball_detected or not self.ball_centered():
                    # On tourne purement (pas de linéaire) jusqu'à centrer la balle
                    twist.linear.x = 0.0
                    twist.angular.z = -self.rotate_speed if self.rotate_direction == "left" else self.rotate_speed
                    self.get_logger().info("CHECK_ALIGNMENT: rotation pour centrer la balle")

                    # Débug affichage
                    ####
                    if self.goal_centered() and self.ball_centered():
                        temp = 1
                    else:
                        temp = 0
                    ####

                elif self.ball_detected and self.ball_centered():
                    # 2) Balle centrée, on regarde le but
                    if self.goal_x is None:
                        # Si but non détecté, on relance une orbite complète
                        self.get_logger().info("CHECK_ALIGNMENT: but perdu, reprise orbite")
                        self.turning_phase = 'REPOSITION_FRONT'
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                    elif not self.goal_centered():
                        # Si but détécté mais pas centré
                        # But détecté mais pas centré → avance + correction
                        error = self.goal_x - (self.image_width // 2)
                        twist.linear.x = self.linear_speed
                        twist.angular.z = -0.002 * error
                        self.get_logger().info(f"CHECK_ALIGNMENT: avance+corrige pour centrer but (err={error})")
                    else:
                        # Balle et but centrés → passage au tir
                        self.get_logger().warn("Alignement atteint: balle et but centrés. On passe au tir")
                        self.state = 'SHOOT'
                        self.push_start_time = time.time()

        elif self.state == 'SHOOT':
            if self.goal_left_edge is not None and self.goal_right_edge is not None:
                self.get_logger().info(f"SHOOT : Centré ! ON FONCE !")

                # Reset du timer de perte de but
                self.lost_goal_start_time = None

                center_x = self.image_width // 2
                error = self.goal_x - center_x

                twist.linear.x = self.linear_speed
                twist.angular.z = -0.002 * error
                self.get_logger().info(f"SHOOT : en approche du but avec correction (err={error})")

            else:
                # Si les poteaux ne sont plus visibles
                twist.linear.x = self.linear_speed * 2
                twist.angular.z = 0.0

                if self.lost_goal_start_time is None:
                    self.lost_goal_start_time = now  # commence le chrono
                    self.get_logger().warn("SHOOT : poteaux perdus → début du chrono")
                elif now - self.lost_goal_start_time < 10.0:
                    self.get_logger().info(f"{now - self.lost_goal_start_time} / 3.0")
                    self.get_logger().warn("SHOOT : poteaux perdus → arrêt après quelques secondes")
                else:
                    self.get_logger().warn(f"SHOOT : arrêt")
                    twist.linear.x = 0.0


        self.cmd_pub.publish(twist)



    def timer_callback(self):
        self.control_loop()


def main(args=None):
    rclpy.init(args=args)
    node = GoalBall()
    node.create_timer(0.1, node.timer_callback)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()