import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np
import cv2
import time

class GoalBall(Node):
    def __init__(self):
        super().__init__('goal_ball')

        # Parameters
        self.declare_parameter('rotate_direction', 'left')  # left or right
        self.declare_parameter('linear_speed', 0.1)
        self.declare_parameter('push_duration', 1.0)
        self.declare_parameter('ball_reposition_distance', 0.85)
        self.declare_parameter('stop_interval', 1.5)
        self.declare_parameter('rotate_speed', 0.25)

        self.rotate_direction = self.get_parameter('rotate_direction').get_parameter_value().string_value
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.push_duration = self.get_parameter('push_duration').get_parameter_value().double_value
        self.ball_reposition_distance = self.get_parameter('ball_reposition_distance').get_parameter_value().double_value
        self.stop_interval = self.get_parameter('stop_interval').get_parameter_value().double_value
        self.rotate_speed = self.get_parameter('rotate_speed').get_parameter_value().double_value

        # Subscribers and publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.bridge = CvBridge()

        # Internal state
        self.ball_detected = False
        self.ball_x = 0
        self.ball_y = 0
        self.image_width = 0
        self.image_height = 0

        self.red_objects = []

        self.lidar_ranges = []

        self.state = 'SEARCH_BALL'
        self.push_start_time = None
        self.rotation_last_stop_time = time.time()

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
        self.red_objects = []
        for c in contours:
            M = cv2.moments(c)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                self.red_objects.append(cx)

    def scan_callback(self, msg):
        self.lidar_ranges = list(msg.ranges)

    def goal_centered(self):
        if len(self.red_objects) >= 2:
            self.red_objects.sort()
            left = self.red_objects[0]
            right = self.red_objects[-1]
            goal_center = (left + right) // 2
            image_center = self.image_width // 2
            return abs(goal_center - image_center) < self.image_width * 0.1
        return False
    
    def goal_alignment_offset(self):
        """Retourne la différence entre le centre de l’image et le centre du but (si visible)."""
        if len(self.red_objects) >= 2:
            self.red_objects.sort()
            goal_center = (self.red_objects[0] + self.red_objects[-1]) // 2
            image_center = self.image_width // 2
            return goal_center - image_center  # positif = but à droite, négatif = à gauche
        return None

    def ball_centered(self):
        if not self.ball_detected:
            return False
        center_x = self.image_width // 2
        return abs(self.ball_x - center_x) < self.image_width * 0.1

    def control_loop(self):
        twist = Twist()
        now = time.time()

        if self.state == 'SEARCH_BALL':
            if self.ball_detected:
                if self.ball_y > self.image_height * self.ball_reposition_distance:
                    direction_text = "la gauche" if self.rotate_direction == "left" else "la droite"
                    self.get_logger().info(f"Balle proche, on la pousse vers {direction_text}")
                    self.push_start_time = now
                    self.state = 'PUSH_BALL'
                else:
                    twist.linear.x = self.linear_speed
                    center_x = self.image_width // 2
                    offset_x = self.ball_x - center_x
                    twist.angular.z = -0.002 * offset_x  # Coefficient à ajuster selon réactivité
                    self.get_logger().info("Balle détectée mais trop loin, on s'en approche")
            else:
                twist.angular.z = 0.2

        elif self.state == 'PUSH_BALL':
            if now - self.push_start_time < self.push_duration:
                direction_text = "la gauche" if self.rotate_direction == "left" else "la droite"
                self.get_logger().info(f"On la pousse vers {direction_text}")
                twist.linear.x = self.linear_speed * 0.5
                twist.angular.z = 0.2 if self.rotate_direction == 'left' else -0.2
            else:
                self.get_logger().info("Fin de poussée, repositionnement autour de la balle")
                self.state = 'TURNING_BALL'
                self.rotation_last_stop_time = now

        elif self.state == 'TURNING_BALL':
            if now - self.rotation_last_stop_time > self.stop_interval:
                self.rotation_last_stop_time = now
                if self.ball_centered() and self.goal_centered():
                    self.get_logger().info("Alignement parfait : balle + but centrés, tir !")
                    self.state = 'SHOOT'
                    self.push_start_time = now
                else:
                    self.get_logger().info("Pause : alignement incomplet (balle ou but)")
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
            else:
                twist.linear.x = self.linear_speed
                twist.angular.z = self.rotate_speed if self.rotate_direction == 'left' else -self.rotate_speed

        elif self.state == 'SHOOT':
            if now - self.push_start_time < 2.0:
                twist.linear.x = self.linear_speed * 1.5
                twist.angular.z = 0.0
            else:
                self.get_logger().info("Tir terminé, arrêt du robot")
                self.state = 'STOP'

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
