import math
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class Evitement(Node):

    def __init__(self):
        super().__init__('evitement')

        # ==================================================
        # PARAMETRES DE DETECTION
        # ==================================================

        self.distance_securite = 0.90
        self.distance_urgence = 0.30


        # ==================================================
        # PARAMETRES D'EVITEMENT
        # ==================================================

        # Angle imposé lors du premier virage
        self.angle_evitement = math.radians(25.0)

        # Distance minimale parcourue après le virage
        # avant d'autoriser le retour
        self.distance_degagement = 0.50


        # ==================================================
        # VITESSES
        # ==================================================

        # Vitesse validée avec notre régulateur PD
        self.vitesse_avance = -0.15

        # Pendant le virage, on ralentit volontairement
        self.vitesse_evitement = -0.08

        # Après le virage : maintien du nouveau cap
        self.vitesse_degagement = -0.15

        # Retour vers le cap du couloir
        self.vitesse_retour = -0.15

        # Rotation imposée pendant EVITE_GAUCHE/DROITE
        self.vitesse_rotation = 0.70


        # ==================================================
        # REGULATEUR PD VALIDE
        # ==================================================

        self.kp = 7.0
        self.kd = 3.0

        self.angular_max = 2.0

        self.erreur_precedente = None
        self.temps_precedent = None


        # ==================================================
        # ETAT DU ROBOT
        # ==================================================

        self.etat = 'AVANCE'

        self.direction_evitement = None

        self.yaw_actuel = None

        self.x_actuel = None
        self.y_actuel = None


        # Cap initial du robot / couloir
        self.cap_couloir = None

        # Cap pris après le virage d'évitement
        self.cap_degagement = None

        # Orientation au début du virage
        self.yaw_debut_evitement = None

        # Position au début du dégagement
        self.x_debut_degagement = None
        self.y_debut_degagement = None


        # ==================================================
        # DONNEES DE PERCEPTION
        # ==================================================

        self.dG = float('inf')
        self.dC = float('inf')
        self.dD = float('inf')

        self.obstacles_recus = False


        # ==================================================
        # ROS
        # ==================================================

        self.subscription_obstacles = self.create_subscription(
            Float32MultiArray,
            '/obstacles',
            self.obstacles_callback,
            10
        )

        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.publisher_cmd = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Boucle de contrôle à 20 Hz
        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.compteur_log = 0

        self.get_logger().info(
            'Evitement avec régulation PD Kp=7 Kd=3 démarré'
        )


    # ======================================================
    # QUATERNION -> YAW
    # ======================================================

    def quaternion_to_yaw(self, x, y, z, w):

        siny_cosp = 2.0 * (
            w * z + x * y
        )

        cosy_cosp = 1.0 - 2.0 * (
            y * y + z * z
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp
        )


    # ======================================================
    # NORMALISATION ANGLE
    # ======================================================

    def normaliser_angle(self, angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle


    # ======================================================
    # RESET DU PD
    # ======================================================

    def reset_pd(self):

        self.erreur_precedente = None
        self.temps_precedent = time.monotonic()


    # ======================================================
    # ODOMETRIE
    # ======================================================

    def odom_callback(self, msg):

        # Position
        self.x_actuel = msg.pose.pose.position.x
        self.y_actuel = msg.pose.pose.position.y

        # Orientation
        q = msg.pose.pose.orientation

        self.yaw_actuel = self.quaternion_to_yaw(
            q.x,
            q.y,
            q.z,
            q.w
        )

        # Première mesure
        if self.cap_couloir is None:

            self.cap_couloir = self.yaw_actuel
            self.cap_degagement = self.yaw_actuel

            self.reset_pd()

            self.get_logger().info(
                f'Cap initial = '
                f'{math.degrees(self.cap_couloir):.2f} deg'
            )


    # ======================================================
    # PERCEPTION
    # ======================================================

    def obstacles_callback(self, msg):

        if len(msg.data) < 3:
            return

        self.dG = msg.data[0]
        self.dC = msg.data[1]
        self.dD = msg.data[2]

        self.obstacles_recus = True


    # ======================================================
    # REGULATION PD
    # ======================================================

    def regulation_pd(self, cap_reference):

        if self.yaw_actuel is None:
            return 0.0

        maintenant = time.monotonic()

        if self.temps_precedent is None:

            self.temps_precedent = maintenant
            return 0.0

        dt = (
            maintenant
            - self.temps_precedent
        )

        self.temps_precedent = maintenant

        if dt <= 0.0 or dt > 0.5:
            dt = 0.05


        # --------------------------------------------------
        # Erreur
        # --------------------------------------------------

        erreur = self.normaliser_angle(
            cap_reference
            - self.yaw_actuel
        )


        # --------------------------------------------------
        # Dérivée
        # --------------------------------------------------

        if self.erreur_precedente is None:

            derivee_erreur = 0.0

        else:

            derivee_erreur = (
                erreur
                - self.erreur_precedente
            ) / dt


        self.erreur_precedente = erreur


        # --------------------------------------------------
        # PD
        #
        # Signe inversé validé sur notre robot
        # --------------------------------------------------

        terme_p = (
            -self.kp
            * erreur
        )

        terme_d = (
            -self.kd
            * derivee_erreur
        )

        correction = (
            terme_p
            +
            terme_d
        )


        # --------------------------------------------------
        # Saturation
        # --------------------------------------------------

        if correction > self.angular_max:

            correction = self.angular_max

        elif correction < -self.angular_max:

            correction = -self.angular_max


        return correction


    # ======================================================
    # CHOIX DU COTE LIBRE
    # ======================================================

    def choisir_direction(self):

        if self.dG > self.dD:

            return 'GAUCHE'

        else:

            return 'DROITE'


    # ======================================================
    # DEMARRAGE EVITEMENT
    # ======================================================

    def demarrer_evitement(self):

        self.direction_evitement = (
            self.choisir_direction()
        )

        self.yaw_debut_evitement = (
            self.yaw_actuel
        )

        self.reset_pd()


        if self.direction_evitement == 'GAUCHE':

            self.etat = 'EVITE_GAUCHE'

            self.get_logger().info(
                'Obstacle détecté -> VIRAGE GAUCHE'
            )

        else:

            self.etat = 'EVITE_DROITE'

            self.get_logger().info(
                'Obstacle détecté -> VIRAGE DROITE'
            )


    # ======================================================
    # DISTANCE PARCOURUE EN DEGAGEMENT
    # ======================================================

    def distance_parcourue_degagement(self):

        if (
            self.x_debut_degagement is None
            or self.y_debut_degagement is None
            or self.x_actuel is None
            or self.y_actuel is None
        ):

            return 0.0


        dx = (
            self.x_actuel
            - self.x_debut_degagement
        )

        dy = (
            self.y_actuel
            - self.y_debut_degagement
        )


        return math.sqrt(
            dx * dx
            +
            dy * dy
        )


    # ======================================================
    # BOUCLE PRINCIPALE
    # ======================================================

    def control_loop(self):

        if (
            not self.obstacles_recus
            or self.yaw_actuel is None
            or self.cap_couloir is None
            or self.x_actuel is None
            or self.y_actuel is None
        ):

            return


        commande = Twist()


        # ==================================================
        # 1. AVANCE
        # ==================================================

        if self.etat == 'AVANCE':

            # Route libre
            if self.dC > self.distance_securite:

                commande.linear.x = (
                    self.vitesse_avance
                )

                commande.angular.z = (
                    self.regulation_pd(
                        self.cap_couloir
                    )
                )


            # Obstacle
            else:

                # Aucun côté suffisamment libre
                if (
                    self.dG < self.distance_securite
                    and
                    self.dD < self.distance_securite
                ):

                    commande.linear.x = 0.0
                    commande.angular.z = 0.0

                    self.get_logger().warning(
                        'Aucun passage sûr -> STOP'
                    )

                else:

                    self.demarrer_evitement()

                    commande.linear.x = 0.0
                    commande.angular.z = 0.0


        # ==================================================
        # 2. VIRAGE GAUCHE
        # ==================================================

        elif self.etat == 'EVITE_GAUCHE':

            commande.linear.x = (
                self.vitesse_evitement
            )

            # Convention du robot :
            # négatif = gauche
            commande.angular.z = (
                -self.vitesse_rotation
            )


            angle_parcouru = abs(
                self.normaliser_angle(
                    self.yaw_actuel
                    - self.yaw_debut_evitement
                )
            )


            if (
                angle_parcouru
                >= self.angle_evitement
            ):

                self.etat = 'DEGAGE_GAUCHE'

                self.cap_degagement = (
                    self.yaw_actuel
                )

                self.x_debut_degagement = (
                    self.x_actuel
                )

                self.y_debut_degagement = (
                    self.y_actuel
                )

                self.reset_pd()

                self.get_logger().info(
                    '25 deg atteints -> DEGAGEMENT GAUCHE'
                )


        # ==================================================
        # 3. VIRAGE DROITE
        # ==================================================

        elif self.etat == 'EVITE_DROITE':

            commande.linear.x = (
                self.vitesse_evitement
            )

            # positif = droite
            commande.angular.z = (
                self.vitesse_rotation
            )


            angle_parcouru = abs(
                self.normaliser_angle(
                    self.yaw_actuel
                    - self.yaw_debut_evitement
                )
            )


            if (
                angle_parcouru
                >= self.angle_evitement
            ):

                self.etat = 'DEGAGE_DROITE'

                self.cap_degagement = (
                    self.yaw_actuel
                )

                self.x_debut_degagement = (
                    self.x_actuel
                )

                self.y_debut_degagement = (
                    self.y_actuel
                )

                self.reset_pd()

                self.get_logger().info(
                    '25 deg atteints -> DEGAGEMENT DROITE'
                )


        # ==================================================
        # 4. DEGAGEMENT GAUCHE
        # ==================================================

        elif self.etat == 'DEGAGE_GAUCHE':

            commande.linear.x = (
                self.vitesse_degagement
            )

            # On maintient précisément
            # le cap obtenu après le virage
            commande.angular.z = (
                self.regulation_pd(
                    self.cap_degagement
                )
            )


            distance = (
                self.distance_parcourue_degagement()
            )


            # Sécurité
            if self.dC < self.distance_urgence:

                commande.linear.x = 0.0
                commande.angular.z = 0.0

                self.get_logger().warning(
                    'Obstacle trop proche pendant DEGAGEMENT'
                )


            else:

                # Après évitement par la gauche,
                # l'obstacle doit être suffisamment
                # dégagé du centre et du côté droit.
                obstacle_depasse = (
                    self.dC > self.distance_securite
                    and
                    self.dD > self.distance_securite
                )


                if (
                    distance
                    >= self.distance_degagement
                    and
                    obstacle_depasse
                ):

                    self.etat = 'RETOUR'

                    self.reset_pd()

                    self.get_logger().info(
                        f'Obstacle dépassé après '
                        f'{distance:.2f} m -> RETOUR'
                    )


        # ==================================================
        # 5. DEGAGEMENT DROITE
        # ==================================================

        elif self.etat == 'DEGAGE_DROITE':

            commande.linear.x = (
                self.vitesse_degagement
            )

            commande.angular.z = (
                self.regulation_pd(
                    self.cap_degagement
                )
            )


            distance = (
                self.distance_parcourue_degagement()
            )


            if self.dC < self.distance_urgence:

                commande.linear.x = 0.0
                commande.angular.z = 0.0

                self.get_logger().warning(
                    'Obstacle trop proche pendant DEGAGEMENT'
                )


            else:

                # Après évitement par la droite,
                # on vérifie centre + gauche.
                obstacle_depasse = (
                    self.dC > self.distance_securite
                    and
                    self.dG > self.distance_securite
                )


                if (
                    distance
                    >= self.distance_degagement
                    and
                    obstacle_depasse
                ):

                    self.etat = 'RETOUR'

                    self.reset_pd()

                    self.get_logger().info(
                        f'Obstacle dépassé après '
                        f'{distance:.2f} m -> RETOUR'
                    )


        # ==================================================
        # 6. RETOUR
        # ==================================================

        elif self.etat == 'RETOUR':

            commande.linear.x = (
                self.vitesse_retour
            )

            commande.angular.z = (
                self.regulation_pd(
                    self.cap_couloir
                )
            )


            erreur_retour = (
                self.normaliser_angle(
                    self.cap_couloir
                    - self.yaw_actuel
                )
            )


            # Sécurité pendant retour
            if self.dC < self.distance_urgence:

                commande.linear.x = 0.0
                commande.angular.z = 0.0

                self.get_logger().warning(
                    'Obstacle proche pendant RETOUR'
                )


            # Cap retrouvé
            elif abs(
                math.degrees(erreur_retour)
            ) < 2.0:

                self.etat = 'AVANCE'

                self.direction_evitement = None

                self.reset_pd()


                commande.linear.x = (
                    self.vitesse_avance
                )

                commande.angular.z = (
                    self.regulation_pd(
                        self.cap_couloir
                    )
                )


                self.get_logger().info(
                    'Cap initial retrouvé -> AVANCE'
                )


        # ==================================================
        # PUBLICATION
        # ==================================================

        self.publisher_cmd.publish(
            commande
        )


        # ==================================================
        # LOG
        # ==================================================

        self.compteur_log += 1


        if self.compteur_log >= 10:

            self.compteur_log = 0


            erreur_deg = math.degrees(
                self.normaliser_angle(
                    self.cap_couloir
                    - self.yaw_actuel
                )
            )


            if (
                self.etat == 'DEGAGE_GAUCHE'
                or
                self.etat == 'DEGAGE_DROITE'
            ):

                distance = (
                    self.distance_parcourue_degagement()
                )

            else:

                distance = 0.0


            self.get_logger().info(

                f'ETAT={self.etat} | '

                f'G={self.dG:.2f} | '
                f'C={self.dC:.2f} | '
                f'D={self.dD:.2f} | '

                f'Yaw='
                f'{math.degrees(self.yaw_actuel):.2f} deg | '

                f'Erreur='
                f'{erreur_deg:.2f} deg | '

                f'Degagement='
                f'{distance:.2f} m | '

                f'vx='
                f'{commande.linear.x:.3f} | '

                f'wz='
                f'{commande.angular.z:.3f}'
            )


# ==========================================================
# MAIN
# ==========================================================

def main(args=None):

    rclpy.init(args=args)

    node = Evitement()


    try:

        rclpy.spin(node)


    except KeyboardInterrupt:

        pass


    finally:

        if rclpy.ok():

            stop = Twist()

            stop.linear.x = 0.0
            stop.angular.z = 0.0

            node.publisher_cmd.publish(
                stop
            )


        node.destroy_node()


        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':
    main()
