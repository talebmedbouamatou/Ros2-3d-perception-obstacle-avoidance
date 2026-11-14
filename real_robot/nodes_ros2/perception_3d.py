import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

import numpy as np


class Perception3D(Node):

    def __init__(self):
        super().__init__('perception_3d')

        # Abonnement à l'image profondeur de la HP60C
        self.subscription = self.create_subscription(
            Image,
            '/ascamera_hp60c/camera_publisher/depth0/image_raw',
            self.depth_callback,
            qos_profile_sensor_data
        )

        # Publication des distances
        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/obstacles',
            10
        )

        self.get_logger().info('Noeud perception_3d demarre')


    def depth_callback(self, msg):

        # Vérification de l'encodage
        if msg.encoding != '16UC1':
            self.get_logger().warn(
                f'Encodage inattendu : {msg.encoding}'
            )
            return

        # ==========================================
        # 1. Conversion Image ROS -> NumPy
        # ==========================================

        depth_mm = np.frombuffer(
            msg.data,
            dtype=np.uint16
        ).reshape(msg.height, msg.width)

        # Conversion millimètres -> mètres
        depth = depth_mm.astype(np.float32) / 1000.0

        h, w = depth.shape

        # ==========================================
        # 2. ROI centrale
        # ==========================================

        y_min = int(h * 0.35)
        y_max = int(h * 0.65)

        x_min = int(w * 0.20)
        x_max = int(w * 0.80)

        roi = depth[y_min:y_max, x_min:x_max]

        # ==========================================
        # 3. Division Gauche / Centre / Droite
        # ==========================================

        largeur_roi = roi.shape[1]
        tiers = largeur_roi // 3

        zone_gauche = roi[:, 0:tiers]
        zone_centre = roi[:, tiers:2 * tiers]
        zone_droite = roi[:, 2 * tiers:largeur_roi]

        # ==========================================
        # 4. Calcul des distances
        # ==========================================

        dG = self.calcul_distance(zone_gauche)
        dC = self.calcul_distance(zone_centre)
        dD = self.calcul_distance(zone_droite)

        # ==========================================
        # 5. Publication sur /obstacles
        # ==========================================

        msg_obstacles = Float32MultiArray()

        msg_obstacles.data = [
            float(dG) if dG is not None else float('nan'),
            float(dC) if dC is not None else float('nan'),
            float(dD) if dD is not None else float('nan')
        ]

        self.publisher.publish(msg_obstacles)

        # ==========================================
        # 6. Affichage couleur dans le terminal
        # ==========================================

        texte_gauche = self.affichage_zone('GAUCHE', dG)
        texte_centre = self.affichage_zone('CENTRE', dC)
        texte_droite = self.affichage_zone('DROITE', dD)

        print(
            f'{texte_gauche} | '
            f'{texte_centre} | '
            f'{texte_droite}',
            flush=True
        )


    def calcul_distance(self, zone):

        # On garde seulement les valeurs valides
        valeurs = zone[
            np.isfinite(zone) &
            (zone > 0.10) &
            (zone < 4.0)
        ]

        # Aucune mesure valide
        if valeurs.size == 0:
            return None

        # Médiane
        return float(np.median(valeurs))


    def affichage_zone(self, nom, distance):

        ROUGE = '\033[91m'
        VERT = '\033[92m'
        RESET = '\033[0m'

        if distance is None:
            return f'{ROUGE}{nom} = HORS PLAGE{RESET}'

        return f'{VERT}{nom} = {distance:.2f} m{RESET}'


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
