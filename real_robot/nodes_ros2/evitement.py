import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

import math


class Evitement(Node):

    def __init__(self):
        super().__init__('evitement')

        # =====================================================
        # PARAMETRES
        # =====================================================

        # Distance minimale de securite devant le robot
        self.distance_securite = 0.50

        # Derniere commande envoyee
        self.derniere_commande = None

        # =====================================================
        # ABONNEMENT AUX DISTANCES DE PERCEPTION_3D
        # =====================================================

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/obstacles',
            self.obstacles_callback,
            10
        )

        # =====================================================
        # PUBLICATION DES COMMANDES
        # =====================================================

        self.publisher_commande = self.create_publisher(
            String,
            '/commande_moteur',
            10
        )

        self.get_logger().info(
            'Noeud evitement demarre'
        )


    # =========================================================
    # RECEPTION DES DISTANCES
    # =========================================================

    def obstacles_callback(self, msg):

        # On attend :
        # [distance_gauche, distance_centre, distance_droite]

        if len(msg.data) < 3:
            self.get_logger().warn(
                'Message /obstacles incomplet'
            )
            return

        dG = float(msg.data[0])
        dC = float(msg.data[1])
        dD = float(msg.data[2])

        # =====================================================
        # 1. VERIFIER SI LE CENTRE EST LIBRE
        # =====================================================

        if (
            self.distance_valide(dC)
            and dC > self.distance_securite
        ):
            self.envoyer_commande('AVANCE')
            return

        # =====================================================
        # 2. CENTRE BLOQUE OU HORS PLAGE
        # =====================================================

        if self.distance_valide(dC):
            self.get_logger().info(
                f'Obstacle devant : {dC:.2f} m'
            )
        else:
            self.get_logger().warn(
                'Centre HORS PLAGE -> recherche passage lateral'
            )

        # =====================================================
        # 3. VERIFIER LES DEUX COTES
        # =====================================================

        gauche_libre = (
            self.distance_valide(dG)
            and dG > self.distance_securite
        )

        droite_libre = (
            self.distance_valide(dD)
            and dD > self.distance_securite
        )

        # =====================================================
        # 4. LES DEUX COTES SONT LIBRES
        # =====================================================

        if gauche_libre and droite_libre:

            if dG >= dD:

                self.get_logger().info(
                    f'Choix GAUCHE : '
                    f'G={dG:.2f} m > D={dD:.2f} m'
                )

                self.envoyer_commande('GAUCHE')

            else:

                self.get_logger().info(
                    f'Choix DROITE : '
                    f'D={dD:.2f} m > G={dG:.2f} m'
                )

                self.envoyer_commande('DROITE')

            return

        # =====================================================
        # 5. SEULEMENT GAUCHE EST LIBRE
        # =====================================================

        if gauche_libre:

            self.get_logger().info(
                'Seul le cote GAUCHE est libre'
            )

            self.envoyer_commande('GAUCHE')

            return

        # =====================================================
        # 6. SEULEMENT DROITE EST LIBRE
        # =====================================================

        if droite_libre:

            self.get_logger().info(
                'Seul le cote DROITE est libre'
            )

            self.envoyer_commande('DROITE')

            return

        # =====================================================
        # 7. AUCUNE DIRECTION N'EST LIBRE
        # =====================================================

        self.get_logger().warn(
            'Aucun passage libre -> STOP'
        )

        self.envoyer_commande('STOP')


    # =========================================================
    # VERIFICATION D'UNE DISTANCE
    # =========================================================

    def distance_valide(self, distance):

        return math.isfinite(distance)


    # =========================================================
    # PUBLICATION DE LA COMMANDE
    # =========================================================

    def envoyer_commande(self, commande):

        if commande == self.derniere_commande:
            return

        msg = String()
        msg.data = commande

        self.publisher_commande.publish(msg)

        self.derniere_commande = commande

        self.get_logger().info(
            f'COMMANDE -> {commande}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = Evitement()

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
