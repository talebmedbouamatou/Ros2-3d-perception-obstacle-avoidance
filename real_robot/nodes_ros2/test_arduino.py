import rclpy
from rclpy.node import Node

import serial
import time


class TestArduino(Node):

    def __init__(self):
        super().__init__('test_arduino')

        port = '/dev/ttyACM0'
        baudrate = 115200

        self.get_logger().info(
            f'Ouverture du port {port}...'
        )

        try:
            arduino = serial.Serial(
                port,
                baudrate,
                timeout=3
            )

        except serial.SerialException as e:
            self.get_logger().error(
                f'Impossible d ouvrir le port : {e}'
            )
            return

        self.get_logger().info(
            'Port serie ouvert avec succes'
        )

        # Arduino Mega redemarre souvent lorsqu'on ouvre
        # la connexion USB serie
        time.sleep(2)

        # Nettoyage d'eventuels anciens messages
        arduino.reset_input_buffer()

        # ==========================================
        # ENVOI DE LA COMMANDE
        # ==========================================

        self.get_logger().info(
            'Envoi de la commande AVANCE'
        )

        arduino.write(b'AVANCE\n')
        arduino.flush()

        # ==========================================
        # ATTENTE DE LA REPONSE ARDUINO
        # ==========================================

        debut = time.time()

        communication_ok = False

        while time.time() - debut < 5:

            if arduino.in_waiting > 0:

                reponse = arduino.readline().decode(
                    'utf-8',
                    errors='ignore'
                ).strip()

                if reponse:

                    self.get_logger().info(
                        f'Arduino -> {reponse}'
                    )

                if reponse == 'ACK:AVANCE':

                    communication_ok = True

                if reponse == 'TEST_TERMINE':
                    break

        # ==========================================
        # RESULTAT
        # ==========================================

        if communication_ok:

            self.get_logger().info(
                'COMMUNICATION SERIE FONCTIONNELLE'
            )

        else:

            self.get_logger().error(
                'Aucune confirmation recue de Arduino'
            )

        arduino.close()

        self.get_logger().info(
            'Port serie ferme'
        )


def main(args=None):

    rclpy.init(args=args)

    node = TestArduino()

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
