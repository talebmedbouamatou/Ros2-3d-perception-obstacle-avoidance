import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class TestLigneDroite(Node):

    def __init__(self):
        super().__init__('test_ligne_droite')

        # ==================================================
        # PARAMETRES
        # ==================================================

        # Chez notre robot :
        # valeur négative = avance
        self.vitesse = -0.15

        # Gains du correcteur PD
        self.kp = 7.0
        self.kd = 3.0

        # Pas d'intégrale
        self.ki = 0.0

        # Saturation de la commande angulaire
        self.angular_max = 2.0

        # Intégrale désactivée
        self.integrale_max = 0.0


        # ==================================================
        # VARIABLES DU REGULATEUR
        # ==================================================

        self.yaw_reference = None

        self.erreur_integrale = 0.0

        self.erreur_precedente = None

        self.temps_precedent = None


        # ==================================================
        # ROS
        # ==================================================

        self.publisher_cmd = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )


        self.get_logger().info(
            'Test ligne droite PD démarré'
        )

        self.get_logger().info(
            f'Vitesse = {self.vitesse:.2f} m/s | '
            f'Kp = {self.kp:.2f} | '
            f'Kd = {self.kd:.2f}'
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
    # CALLBACK ODOM
    # ======================================================

    def odom_callback(self, msg):

        # --------------------------------------------------
        # Lecture orientation
        # --------------------------------------------------

        q = msg.pose.pose.orientation

        yaw = self.quaternion_to_yaw(
            q.x,
            q.y,
            q.z,
            q.w
        )


        maintenant = time.monotonic()


        # --------------------------------------------------
        # Première mesure
        # --------------------------------------------------

        if self.yaw_reference is None:

            self.yaw_reference = yaw

            self.temps_precedent = maintenant

            self.erreur_integrale = 0.0

            self.erreur_precedente = 0.0


            self.get_logger().info(
                f'Cap de référence enregistré : '
                f'{math.degrees(self.yaw_reference):.2f} deg'
            )

            return


        # --------------------------------------------------
        # Temps écoulé
        # --------------------------------------------------

        dt = (
            maintenant
            - self.temps_precedent
        )

        self.temps_precedent = maintenant


        if dt <= 0.0 or dt > 0.5:
            dt = 0.01


        # --------------------------------------------------
        # ERREUR DE CAP
        # --------------------------------------------------

        erreur = self.normaliser_angle(
            self.yaw_reference - yaw
        )


        # --------------------------------------------------
        # TERME DERIVE DIRECT
        # --------------------------------------------------

        derivee_erreur = (
            erreur
            - self.erreur_precedente
        ) / dt


        self.erreur_precedente = erreur


        # --------------------------------------------------
        # TERME INTEGRAL
        # Ki = 0 donc sans effet
        # --------------------------------------------------

        self.erreur_integrale += (
            erreur * dt
        )


        if (
            self.erreur_integrale
            > self.integrale_max
        ):

            self.erreur_integrale = (
                self.integrale_max
            )


        elif (
            self.erreur_integrale
            < -self.integrale_max
        ):

            self.erreur_integrale = (
                -self.integrale_max
            )


        # --------------------------------------------------
        # REGULATEUR PD
        # Signe inversé adapté au robot
        # --------------------------------------------------

        terme_p = (
            -self.kp
            * erreur
        )


        terme_d = (
            -self.kd
            * derivee_erreur
        )


        terme_i = (
            -self.ki
            * self.erreur_integrale
        )


        correction = (
            terme_p
            +
            terme_d
            +
            terme_i
        )


        # --------------------------------------------------
        # SATURATION
        # --------------------------------------------------

        if correction > self.angular_max:

            correction = self.angular_max


        elif correction < -self.angular_max:

            correction = -self.angular_max


        # --------------------------------------------------
        # COMMANDE ROBOT
        # --------------------------------------------------

        commande = Twist()

        commande.linear.x = self.vitesse

        commande.angular.z = correction


        self.publisher_cmd.publish(
            commande
        )


        # --------------------------------------------------
        # AFFICHAGE
        # --------------------------------------------------

        self.get_logger().info(

            f'Yaw ref = '
            f'{math.degrees(self.yaw_reference):.2f} deg | '

            f'Yaw = '
            f'{math.degrees(yaw):.2f} deg | '

            f'Erreur = '
            f'{math.degrees(erreur):.2f} deg | '

            f'P = {terme_p:.4f} | '

            f'D = {terme_d:.4f} | '

            f'Correction = {correction:.4f}'
        )


# ==========================================================
# MAIN
# ==========================================================

def main(args=None):

    rclpy.init(args=args)

    node = TestLigneDroite()


    try:

        rclpy.spin(node)


    except KeyboardInterrupt:

        pass


    finally:

        # --------------------------------------------------
        # Arrêt du robot
        # --------------------------------------------------

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
