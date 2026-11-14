import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import serial
import time


class CommandeArduino(Node):

    def __init__(self):
        super().__init__('commande_arduino')

        # =====================================================
        # PARAMETRES PORT SERIE
        # =====================================================

        self.port = '/dev/ttyACM0'
        self.baudrate = 115200

        # Derniere commande envoyee par le Raspberry
        self.derniere_commande = 'AUCUNE'

        # Dernier etat recu depuis Arduino
        self.etat_arduino = 'INCONNU'

        # =====================================================
        # OUVERTURE DU PORT SERIE
        # =====================================================

        try:

            self.arduino = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0
            )

            self.get_logger().info(
                f'Arduino connecte sur {self.port} '
                f'a {self.baudrate} bauds'
            )

            # L'ouverture du port serie peut provoquer
            # un reset de l'Arduino Mega.
            time.sleep(2)

            # Supprimer les anciens caracteres eventuellement
            # presents dans le buffer.
            self.arduino.reset_input_buffer()

        except serial.SerialException as e:

            self.get_logger().error(
                f'Impossible d ouvrir le port serie : {e}'
            )

            self.arduino = None

        # =====================================================
        # ABONNEMENT AUX COMMANDES DE EVITEMENT
        # =====================================================

        self.subscription = self.create_subscription(
            String,
            '/commande_moteur',
            self.commande_callback,
            10
        )

        # =====================================================
        # PUBLICATION DE L'ETAT ARDUINO
        # =====================================================

        # Ce topic sert uniquement au diagnostic / affichage.
        # Il n'est pas utilise pour prendre les decisions
        # d'evitement.
        self.publisher_etat = self.create_publisher(
            String,
            '/etat_arduino',
            10
        )

        # =====================================================
        # LECTURE PERIODIQUE DU PORT SERIE
        # =====================================================

        # Toutes les 20 ms, on regarde si Arduino
        # a envoye un message.
        self.timer_serie = self.create_timer(
            0.02,
            self.lire_arduino
        )

        self.get_logger().info(
            'Noeud commande_arduino demarre'
        )


    # =========================================================
    # COMMANDE RECUE DE EVITEMENT.PY
    # =========================================================

    def commande_callback(self, msg):

        commande = msg.data.strip()

        commandes_valides = [
            'AVANCE',
            'GAUCHE',
            'DROITE',
            'STOP'
        ]

        if commande not in commandes_valides:

            self.get_logger().warn(
                f'Commande inconnue : {commande}'
            )

            return

        if self.arduino is None:

            self.get_logger().error(
                'Arduino non connecte'
            )

            return

        try:

            # =================================================
            # ENVOI VERS ARDUINO
            # =================================================

            message = commande + '\n'

            self.arduino.write(
                message.encode('utf-8')
            )

            self.arduino.flush()

            self.derniere_commande = commande

            # Affichage
            self.afficher_etat()

        except serial.SerialException as e:

            self.get_logger().error(
                f'Erreur lors de l envoi serie : {e}'
            )


    # =========================================================
    # LECTURE DES MESSAGES VENANT DE ARDUINO
    # =========================================================

    def lire_arduino(self):

        if self.arduino is None:
            return

        try:

            while self.arduino.in_waiting > 0:

                ligne = self.arduino.readline()

                if not ligne:
                    return

                message = ligne.decode(
                    'utf-8',
                    errors='ignore'
                ).strip()

                if message == '':
                    continue

                # =============================================
                # MESSAGES AUTORISES DE ARDUINO
                # =============================================

                messages_valides = [
                    'ACK:AVANCE',
                    'ACK:GAUCHE',
                    'ACK:DROITE',
                    'ACK:STOP',
                    'BUSY',
                    'PAUSE',
                    'REPRISE',
                    'TERMINE'
                ]

                if message in messages_valides:

                    self.etat_arduino = message

                    # Publication uniquement pour observation
                    msg_etat = String()
                    msg_etat.data = message

                    self.publisher_etat.publish(msg_etat)

                    # Affichage dans le terminal
                    self.afficher_etat()

                else:

                    self.get_logger().warn(
                        f'Message Arduino inconnu : {message}'
                    )

        except serial.SerialException as e:

            self.get_logger().error(
                f'Erreur lecture serie : {e}'
            )


    # =========================================================
    # AFFICHAGE
    # =========================================================

    def afficher_etat(self):

        self.get_logger().info(
            f'[COMMANDE -> ARDUINO] '
            f'{self.derniere_commande:<8} | '
            f'[ETAT ARDUINO] '
            f'{self.etat_arduino}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = CommandeArduino()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        if node.arduino is not None:

            node.arduino.close()

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()
