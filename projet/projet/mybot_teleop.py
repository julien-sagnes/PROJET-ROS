import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import click
import threading
from numpy import pi

from geometry_msgs.msg import Twist     #Car avec ros2 topic info, on trouve /geometry_msg/msg/Twist
from geometry_msgs.msg import Vector3   #Car avec ros2 topic info, on trouve /geometry_msg/msg/Vector3

class myTeleopNode(Node):

    def __init__(self):
        super().__init__('mybot_teleop')

        self.declare_parameter('linear_scale',2.0)
        self.declare_parameter('angular_scale',pi/2)
        self.declare_parameter('topic_publisher','/cmd_vel')

        self.topic_publisher = self.get_parameter('topic_publisher').get_parameter_value().string_value

        # Define the keycodes
        self.keycode = {'\x1b[A': 'up', '\x1b[B': 'down',   #Flèche sur le Terminal
                        '\x1b[C': 'right', '\x1b[D': 'left', 's': 'stop', 'q': 'quit'}
        self.message = Twist(
            linear = Vector3(x = 0.0,y = 0.0,z = 0.0), 
            angular = Vector3(x = 0.0,y = 0.0,z = 0.0))

            #On peut aussi sans Vector3 avec self.message.linear.x = 0 ou self.linear.angular.y = 0

        self.publisher_ = self.create_publisher(Twist, self.topic_publisher, 10) #Type des topic trouvé avec ros2 topic list -t
                                                                                 #Forme (noms des attributs) : ros2 interface show <nom/du/Type>

    def run(self):
        # Get character from keyboard
        # Mettre la valeur du paramètres dans une valeur

        while True:
            self.get_logger().info('Tapez les flèches directionnelles pour contrôler le robot, s : stopper, q : quitter')

            mykey = click.getchar()
            char = self.keycode[mykey]

            # Prendre les valeurs des paramètres
            self.linear_scale = self.get_parameter('linear_scale').get_parameter_value().double_value
            self.angular_scale = self.get_parameter('angular_scale').get_parameter_value().double_value

            if char == 'up':    # UP key
                self.message.linear.x = self.linear_scale
                self.get_logger().info('Haut')
            if char == 'down':  # DOWN key
                self.message.linear.x = -self.linear_scale
                self.get_logger().info('Bas')
            if char == 'right':  # RIGHT key
                self.message.angular.z = -self.angular_scale
                self.get_logger().info('Droite')
            if char == 'left':  # LEFT
                self.message.angular.z = self.angular_scale
                self.get_logger().info('Gauche')
            if char == 'stop':  # stop
                self.message = Twist(linear = Vector3(x = 0.0,y = 0.0,z = 0.0), angular = Vector3(x = 0.0,y = 0.0,z = 0.0))
                self.get_logger().info('Arrêt du mouvement')
            if char == 'quit':  # qqq
                self.get_logger().info('Quitting...')
                exit()

            self.publisher_.publish(self.message)
            self.message = Twist(linear = Vector3(x = 0.0,y = 0.0,z = 0.0), angular = Vector3(x = 0.0,y = 0.0,z = 0.0))

def main(args=None):
    rclpy.init(args=args)

    # Init node
    teleop_node = myTeleopNode()

    # Pour que les paramètres apparaissent dans ros2 param list
    thread = threading.Thread(target = rclpy.spin, args = (teleop_node,), daemon=True)
    thread.start()

    # Run keyboard reading and corresponding publications
    teleop_node.run()

    # Shut everything down
    teleop_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
