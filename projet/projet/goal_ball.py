import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
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
        self.declare_parameter('rotate_direction', 'left')
        self.declare_parameter('linear_speed', 0.1)
        self.declare_parameter('push_duration', 0.0)
        self.declare_parameter('ball_reposition_distance', 0.55)
        self.declare_parameter('stop_interval', 1.5)
        self.declare_parameter('rotate_speed', 1.0)

        self.rotate_direction = self.get_parameter('rotate_direction').get_parameter_value().string_value
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.push_duration = self.get_parameter('push_duration').get_parameter_value().double_value
        self.ball_reposition_distance = self.get_parameter('ball_reposition_distance').get_parameter_value().double_value
        self.stop_interval = self.get_parameter('stop_interval').get_parameter_value().double_value
        self.rotate_speed = self.get_parameter('rotate_speed').get_parameter_value().double_value

        # ROS
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)

        self.bridge = CvBridge()

        # État interne
        self.ball_detected = False
        self.ball_x = 0
        self.ball_y = 0
        self.image_width = 0
        self.image_height = 0

        self.lidar_ranges = []

        self.state = 'SEARCH_BALL'
        self.turning_phase = None
        self.push_start_time = None
        self.rotation_start_time = None
        self.orbit_start_time = None

        # Orbite de la balle
        coef = 6 # facteur empirique constaté pour la rotation à 90
        self.rotation_duration = (math.pi / 2) / self.rotate_speed * coef  # Durée théorique pour effectuer 90°
        self.orbit_angle = math.pi / 3 * coef  # 60°
        self.orbit_linear_speed = 0.1  # m/s
        self.orbit_angular_speed = self.orbit_linear_speed / self.ball_reposition_distance * coef
        self.orbit_duration = self.orbit_angle / self.orbit_angular_speed

        self.get_logger().info("Node GoalBall started")

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        self.image_width = msg.width
        self.image_height = msg.height

        # Yellow mask for the ball
        lower_yellow = np.array([20, 80, 80])
        upper_yellow = np.array([40, 255, 255])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        yellow_mask = cv2.GaussianBlur(yellow_mask, (5, 5), 0)

        # Red mask for goal posts
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
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
            cv2.circle(cv_image, (self.goal_x, self.image_height // 3), 10, (255, 255, 0), -1) 
            # Affiche une ligne verticale au centre de l'image pour référence
            cv2.line(cv_image, (self.image_width // 2, 0), (self.image_width // 2, self.image_height), (255, 0, 0), 2)  # bleu
            # Affiche une ligne verticale à goal_x
            cv2.line(cv_image, (self.goal_x, 0), (self.goal_x, self.image_height), (255, 255, 0), 2)

        cv2.imshow("Goal Visualization", cv_image)

    def goal_centered(self):
        if self.goal_x is not None:
            image_center = self.image_width // 2
            tolerance = self.image_width * 0.05  # 5% de tolérance
            return abs(self.goal_x - image_center) < tolerance
        return False

    def goal_alignment_offset(self):
        """Retourne la différence entre le centre de l’image et le centre du but (si visible)."""
        if self.goal_x is not None:
            image_center = self.image_width // 2
            return self.goal_x - image_center
        return None

    def ball_centered(self):
        if not self.ball_detected:
            return False
        center_x = self.image_width // 2
        return abs(self.ball_x - center_x) < self.image_width * 0.1 # Le centre du but est dans les 10 % du centre de l'image

    def control_loop(self):
        twist = Twist()
        now = time.time()

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
                self.get_logger().info("Allumage de la caméra...")

        elif self.state == 'PUSH_BALL':
            if now - self.push_start_time < self.push_duration:
                twist.linear.x = self.linear_speed * 0.5
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
                    twist.linear.x = - self.linear_speed * 0.5
                    center_x = self.image_width // 2
                    offset_x = self.ball_x - center_x
                    twist.angular.z = -0.002 * offset_x
                else:
                    self.get_logger().info("Repositionnement terminé. Rotation initiale.")
                    self.turning_phase = 'ROTATE_90'
                    self.rotation_start_time = now

            elif self.turning_phase == 'ROTATE_90':
                if now - self.rotation_start_time < self.rotation_duration:
                    self.get_logger().info(f"Tourner durant {now - self.rotation_start_time} / {self.rotation_duration}.")
                    twist.angular.z = self.rotate_speed if self.rotate_direction == 'left' else -self.rotate_speed
                else:
                    self.get_logger().info("Fin de la rotation. Début de l'orbite.")
                    self.turning_phase = 'ORBIT'
                    self.orbit_start_time = now

            elif self.turning_phase == 'ORBIT':
                if now - self.orbit_start_time < self.orbit_duration:
                    self.get_logger().info(f"Orbite durant {now - self.orbit_start_time} / {self.orbit_duration}.")
                    twist.linear.x = self.orbit_linear_speed
                    twist.angular.z = -self.orbit_angular_speed if self.rotate_direction == 'left' else self.orbit_angular_speed
                else:
                    self.get_logger().info("Pause orbite, on regarde la balle.")
                    self.turning_phase = 'CHECK_ALIGNMENT'
                    self.rotation_start_time = now

            elif self.turning_phase == 'CHECK_ALIGNMENT':
                # 1) Phase de rotation pour centrer la balle
                if not self.ball_detected or not self.ball_centered():
                    # On tourne purement (pas de linéaire) jusqu'à centrer la balle
                    twist.linear.x = 0.0
                    twist.angular.z = -self.rotate_speed if self.rotate_direction == "left" else -self.rotate_speed
                    self.get_logger().info("CHECK_ALIGNMENT: rotation pour centrer la balle")
                else:
                    # 2) Balle centrée, on regarde le but
                    if self.goal_x is None:
                        # Si but non détecté, on relance une orbite complète
                        self.get_logger().info("CHECK_ALIGNMENT: but perdu, reprise orbite")
                        self.turning_phase = 'REPOSITION_FRONT'
                        self.orbit_start_time = time.time()
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                    elif abs(self.goal_x - self.image_width // 2) > self.image_width * 0.3:
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
            # Si on voit deux poteaux rouges
            if len(self.red_objects) >= 2:
                # Calcul de l'écart en pixels
                if self.goal_left_edge is not None and self.goal_right_edge is not None:
                    post_width_px = abs(self.goal_right_edge - self.goal_left_edge)

                target_width = self.image_width * 0.95  # S'approcher du but

                self.get_logger().info(
                    f"SHOOT : distance but = {post_width_px}px / seuil = {target_width:.0f}px"
                )
                # Tant que le but n'est pas assez "large" dans l'image, on avance
                if post_width_px < target_width:
                    twist.linear.x = self.linear_speed * 1.5
                    twist.angular.z = 0.0
                    self.get_logger().info("SHOOT : en approche du but.")
                else:
                    self.get_logger().info("SHOOT : but atteint visuellement, arrêt.")
                    self.state = 'STOP'
            else:
                # Si les poteaux ne sont plus visibles (éventuelle occlusion), on avance prudemment
                twist.linear.x = self.linear_speed * 0.5
                twist.angular.z = 0.0
                self.get_logger().warn("SHOOT : poteaux perdus, retour à SEARCH_BALL.")
                self.state = "SEARCH_BALL"

        else:  # STOP
            twist.linear.x = 0.0
            twist.angular.z = 0.0

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