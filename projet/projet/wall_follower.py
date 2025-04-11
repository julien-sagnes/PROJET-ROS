import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')

        # Souscription aux distances LIDAR déjà traitées avec le noeud lds_distances
        self.subscriber = self.create_subscription(
            Float32MultiArray,
            '/lds_distances',
            self.dist_callback,
            10
        )

        # Publication des commandes de mouvement
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Distances initialisées à infini
        self.front_dist = float('inf')
        self.left_dist = float('inf')
        self.right_dist = float('inf')

        self.declare_parameter('linear_speed', 0.1)
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value

        self.kp = 0.5

        self.get_logger().info("wall_follower_node started.")

    def dist_callback(self, msg):
        # Réception des 3 distances : front, left, right (sans back inutile)
        if len(msg.data) >= 3:
            self.front_dist = msg.data[0]
            self.left_dist = msg.data[1]
            self.right_dist = msg.data[2]
            self.control_loop()

    def control_loop(self):
        twist = Twist()

        # Vérifie si les distances sont valides
        if self.left_dist == float('inf') or self.right_dist == float('inf'):
            self.get_logger().warn("Pas de mur, en attente de données LIDAR...")
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        else:
            # Ajouter une marge de sécurité
            desired_distance = 0.5  # Distance souhaitée par rapport au mur (en mètres)
            error = (self.left_dist - desired_distance) - (self.right_dist - desired_distance)

            # Gérer les cas où le robot est trop proche d'un mur
            if self.left_dist < 0.1 or self.right_dist < 0.1:
                twist.linear.x = self.linear_speed / 2  # Réduire la vitesse
                twist.angular.z = -self.kp * error * 2  # Augmenter la correction angulaire
            else:
                twist.linear.x = self.linear_speed
                twist.angular.z = -self.kp * error  # Négatif car tourne vers côté plus proche

            # Éviter les obstacles frontaux
            if self.front_dist < 0.2:
                twist.linear.x = 0.0  # S'arrêter
                twist.angular.z = 0.5  # Tourner pour éviter l'obstacle

            self.get_logger().info(f"right = {self.right_dist}, left = {self.left_dist}")
            self.get_logger().info(f"error = {error}")
            self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()