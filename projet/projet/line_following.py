import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineFollowingNode(Node):
    def __init__(self):
        super().__init__('line_following_node')

        self.image_subscriber = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()

        self.get_logger().info('Line following node started.')

        # Seuils HSV
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])
        self.lower_green = np.array([30, 100, 100])
        self.upper_green = np.array([90, 255, 255])

    def image_callback(self, img_msg):
        self.get_logger().info('Image received')
        img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Masques couleurs
        mask_red1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask_red2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_green = cv2.inRange(hsv, self.lower_green, self.upper_green)

        # Morphologie
        kernel = np.ones((5, 5), np.uint8)
        mask_red_clean = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
        mask_red_clean = cv2.morphologyEx(mask_red_clean, cv2.MORPH_CLOSE, kernel)
        mask_green_clean = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
        mask_green_clean = cv2.morphologyEx(mask_green_clean, cv2.MORPH_CLOSE, kernel)

        # Contours
        contours_red, _ = cv2.findContours(mask_red_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_green, _ = cv2.findContours(mask_green_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours_red:
            contours_red = [max(contours_red, key=cv2.contourArea)]
            self.get_logger().info('Red contour detected.')
        else:
            self.get_logger().warn('No red contour detected.')

        if contours_green:
            contours_green = [max(contours_green, key=cv2.contourArea)]
            self.get_logger().info('Green contour detected.')
        else:
            self.get_logger().warn('No green contour detected.')

        cv2.drawContours(img, contours_red, -1, (0, 0, 255), 2)
        cv2.drawContours(img, contours_green, -1, (0, 255, 0), 2)

        if contours_red and contours_green:
            M_red = cv2.moments(contours_red[0])
            M_green = cv2.moments(contours_green[0])

            if M_red["m00"] != 0 and M_green["m00"] != 0:
                cx_red = int(M_red["m10"] / M_red["m00"])
                cx_green = int(M_green["m10"] / M_green["m00"])
                cx_center = (cx_red + cx_green) // 2

                self.get_logger().info(f'Red cx: {cx_red}, Green cx: {cx_green}, Center: {cx_center}')

                twist = Twist()
                twist.linear.x = 0.15
                twist.angular.z = (cx_center - img.shape[1] // 2) * 0.002
                self.cmd_vel_publisher.publish(twist)
                self.get_logger().info(f'Moving: linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f}')
            else:
                self.get_logger().warn('Zero division risk in moments. Stopping.')
                self.stop_robot()
        else:
            self.get_logger().warn('Contours missing. Stopping robot.')
            self.stop_robot()

        cv2.imshow('Contours', img)
        cv2.waitKey(1)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info('Robot stopped.')

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()