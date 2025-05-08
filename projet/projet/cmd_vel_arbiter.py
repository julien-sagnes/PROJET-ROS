import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class CmdVelArbiter(Node):
    def __init__(self):
        super().__init__('cmd_vel_arbiter')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.obstacle_msg = None
        self.line_msg = None
        self.obstacle_active = False

        self.create_subscription(Twist, '/cmd_vel_obstacle', self.obstacle_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_line', self.line_callback, 10)
        self.create_subscription(Bool, '/obstacle_active', self.obstacle_flag_callback, 10)

        self.timer = self.create_timer(0.05, self.publish_cmd_vel)
        self.get_logger().info(f"cmd_vel_arbiter activé")

    def obstacle_callback(self, msg):
        self.obstacle_msg = msg

    def line_callback(self, msg):
        self.line_msg = msg

    def obstacle_flag_callback(self, msg):
        self.obstacle_active = msg.data

    def publish_cmd_vel(self):
        if self.obstacle_msg is not None and self.obstacle_active:
            self.cmd_pub.publish(self.obstacle_msg)
            self.get_logger().info('Obstacle détecté - priorité EVITEMENT')
        elif self.line_msg is not None:
            self.cmd_pub.publish(self.line_msg)
            self.get_logger().info('Suivi de ligne actif')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelArbiter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
