#!/usr/bin/env python3

# ==========================================================
# IMPORT BASE ROS2
# ==========================================================
import rclpy
from rclpy.node import Node

# ==========================================================
# SERVICE STANDARD
# ==========================================================
# Lo usiamo solo come "trigger"
# Terminale equivalente:
# ros2 service call /play_motion_service example_interfaces/srv/SetBool "{data: true}"
from example_interfaces.srv import SetBool

# ==========================================================
# IMPORT PER ESEGUIRE COMANDI ROS2
# ==========================================================
import subprocess


# ==========================================================
# SERVICE SERVER
# ==========================================================
class PlayMotionServiceServer(Node):

    def __init__(self):
        super().__init__('play_motion_service_server')

        # Creazione del service
        # Nome: /play_motion_service
        # Tipo: SetBool
        self.service = self.create_service(
            SetBool,
            '/play_motion_service',
            self.callback
        )

        self.get_logger().info("Service server pronto.")

    # ==========================================================
    # CALLBACK SERVICE
    # ==========================================================
    def callback(self, request, response):
        """
        request.data = True/False

        Se True → eseguiamo una motion
        Se False → non facciamo nulla
        """

        # ------------------------------------------------------
        # CASO TRUE → esegui motion
        # ------------------------------------------------------
        if request.data:

            motion_name = "head_tour"

            self.get_logger().info(f"Eseguo motion: {motion_name}")

            # ==================================================
            # EQUIVALENTE TERMINALE:
            # ros2 run play_motion2 run_motion head_tour
            # ==================================================
            result = subprocess.run(
                ['ros2', 'run', 'play_motion2', 'run_motion', motion_name],
                capture_output=True,
                text=True
            )

            # Controllo risultato
            if result.returncode == 0:
                response.success = True
                response.message = f"Motion {motion_name} eseguita!"
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = result.stderr
                self.get_logger().error(response.message)

        # ------------------------------------------------------
        # CASO FALSE → niente
        # ------------------------------------------------------
        else:
            response.success = False
            response.message = "Richiesta falsa, nessuna motion eseguita."
            self.get_logger().info(response.message)

        return response


# ==========================================================
# MAIN
# ==========================================================
def main(args=None):
    rclpy.init(args=args)

    node = PlayMotionServiceServer()

    # Mantiene il nodo attivo
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()