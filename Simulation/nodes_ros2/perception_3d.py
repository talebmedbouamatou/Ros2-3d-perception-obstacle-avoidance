import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

import numpy as np


class Perception3D(Node):

    def __init__(self):
        super().__init__('perception_3d')

        # Topic de la caméra Depth simulée
        self.depth_topic = '/depth_camera/depth_camera/depth/image_raw'

        # Abonnement à l'image Depth
        self.subscription = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )

        # Publication des distances :
        # data = [gauche, centre, droite]
        self.publisher_obstacles = self.create_publisher(
            Float32MultiArray,
            '/obstacles',
            10
        )

        self.get_logger().info('Node perception_3d démarré')
        self.get_logger().info(
            f'Écoute du topic : {self.depth_topic}'
        )
        self.get_logger().info(
            'Publication des distances sur /obstacles'
        )


    def calculer_distance(self, zone):

        # Garder seulement les profondeurs valides
        valeurs_valides = zone[
            np.isfinite(zone) &
            (zone > 0.10) &
            (zone < 4.0)
        ]

        if valeurs_valides.size == 0:
            return float('inf')

        # Distance représentative de la zone
        return float(np.median(valeurs_valides))


    def depth_callback(self, msg):

        # Notre caméra Gazebo fournit du 32FC1
        if msg.encoding != '32FC1':
            self.get_logger().warning(
                f'Encodage inattendu : {msg.encoding}'
            )
            return

        # Conversion du message ROS en matrice NumPy
        depth_image = np.frombuffer(
            msg.data,
            dtype=np.float32
        ).reshape((msg.height, msg.width))

        hauteur = msg.height
        largeur = msg.width

        # Bande horizontale centrale
        y_min = int(hauteur * 0.35)
        y_max = int(hauteur * 0.65)

        # Division gauche / centre / droite
        zone_gauche = depth_image[
            y_min:y_max,
            0:int(largeur / 3)
        ]

        zone_centre = depth_image[
            y_min:y_max,
            int(largeur / 3):int(2 * largeur / 3)
        ]

        zone_droite = depth_image[
            y_min:y_max,
            int(2 * largeur / 3):largeur
        ]

        # Calcul des distances
        dG = self.calculer_distance(zone_gauche)
        dC = self.calculer_distance(zone_centre)
        dD = self.calculer_distance(zone_droite)

        # Affichage
        self.get_logger().info(
            f'GAUCHE = {dG:.2f} m | '
            f'CENTRE = {dC:.2f} m | '
            f'DROITE = {dD:.2f} m'
        )

        # Publication vers le node evitement
        msg_obstacles = Float32MultiArray()

        msg_obstacles.data = [
            dG,
            dC,
            dD
        ]

        self.publisher_obstacles.publish(msg_obstacles)


def main(args=None):

    rclpy.init(args=args)

    node = Perception3D()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
