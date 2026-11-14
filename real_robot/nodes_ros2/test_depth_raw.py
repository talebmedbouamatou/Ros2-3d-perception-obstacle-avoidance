import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data

import numpy as np


class TestDepthRaw(Node):

    def __init__(self):
        super().__init__('test_depth_raw')

        self.subscription = self.create_subscription(
            Image,
            '/ascamera_hp60c/camera_publisher/depth0/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            'Noeud test_depth_raw demarre'
        )


    def image_callback(self, msg):

        # =====================================================
        # VERIFIER LE FORMAT DE L'IMAGE
        # =====================================================

        if msg.encoding != '16UC1':
            self.get_logger().warning(
                f'Format recu : {msg.encoding} au lieu de 16UC1'
            )
            return


        # =====================================================
        # CONVERSION DE L'IMAGE EN TABLEAU NUMPY
        #
        # IMPORTANT :
        # ici on garde les valeurs BRUTES en millimetres.
        # Aucune conversion en metres.
        # Aucun filtrage.
        # =====================================================

        image = np.frombuffer(
            msg.data,
            dtype=np.uint16
        ).reshape(
            msg.height,
            msg.width
        )


        # =====================================================
        # MEME ROI GENERALE QUE NOTRE PERCEPTION
        #
        # Hauteur :
        #   35 % -> 65 %
        #
        # Largeur :
        #   20 % -> 80 %
        # =====================================================

        h = msg.height
        w = msg.width

        y1 = int(h * 0.35)
        y2 = int(h * 0.65)

        x1 = int(w * 0.20)
        x2 = int(w * 0.80)

        roi = image[y1:y2, x1:x2]


        # =====================================================
        # DIVISER EN TROIS ZONES
        # =====================================================

        largeur_roi = roi.shape[1]

        tiers = largeur_roi // 3

        zone_gauche = roi[:, 0:tiers]

        zone_centre = roi[:, tiers:2 * tiers]

        zone_droite = roi[:, 2 * tiers:]


        # =====================================================
        # ANALYSE BRUTE DE LA ZONE CENTRALE
        # =====================================================

        valeurs = zone_centre.flatten()

        minimum = np.min(valeurs)
        maximum = np.max(valeurs)
        mediane = np.median(valeurs)

        nombre_zero = np.sum(valeurs == 0)

        nombre_total = valeurs.size

        pourcentage_zero = (
            nombre_zero / nombre_total
        ) * 100.0


        # =====================================================
        # VALEURS NON NULLES
        # =====================================================

        valeurs_non_nulles = valeurs[valeurs > 0]

        if valeurs_non_nulles.size > 0:

            min_non_nul = np.min(valeurs_non_nulles)

            max_non_nul = np.max(valeurs_non_nulles)

            mediane_non_nulle = np.median(
                valeurs_non_nulles
            )

        else:

            min_non_nul = 0
            max_non_nul = 0
            mediane_non_nulle = 0


        # =====================================================
        # AFFICHAGE
        # =====================================================

        print("\n==============================================")

        print("ZONE CENTRALE - VALEURS BRUTES HP60C")

        print("----------------------------------------------")

        print(f"Resolution image : {msg.width} x {msg.height}")

        print(f"Nombre pixels zone centre : {nombre_total}")

        print("----------------------------------------------")

        print(f"MIN brut          = {minimum} mm")

        print(f"MEDIANE brute     = {mediane} mm")

        print(f"MAX brut          = {maximum} mm")

        print("----------------------------------------------")

        print(f"Nombre de zeros   = {nombre_zero}")

        print(f"Pourcentage zeros = {pourcentage_zero:.2f} %")

        print("----------------------------------------------")

        print(f"MIN non nul       = {min_non_nul} mm")

        print(
            f"MEDIANE non nulle = "
            f"{mediane_non_nulle} mm"
        )

        print(f"MAX non nul       = {max_non_nul} mm")

        print("==============================================")


def main(args=None):

    rclpy.init(args=args)

    node = TestDepthRaw()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
