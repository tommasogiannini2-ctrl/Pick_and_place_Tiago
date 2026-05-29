#!/usr/bin/env python3

# ==========================================================
# IMPORT BASE ROS2
# ==========================================================
import rclpy
from rclpy.node import Node

# ==========================================================
# IMPORT SERVIZI PLAY_MOTION2
# ==========================================================
# Assicurati di aver fatto:
# source /home/rita/FdR_ISI2026/tiago_public_ws/install/setup.bash
# prima di eseguire questo script, così Python trova i messaggi custom.
from play_motion2_msgs.srv import ListMotions, IsMotionReady

# ==========================================================
# IMPORT PER ESEGUIRE MOTION COME COMANDO SHELL
# ==========================================================
import subprocess

# ==========================================================
# NODO CLIENT PER PLAY_MOTION2
# ==========================================================
class PlayMotionClient(Node):
    def __init__(self):
        super().__init__('play_motion_client')

        # Client per il servizio list_motions
        self.list_motions_client = self.create_client(
            ListMotions,
            '/play_motion2/list_motions'
        )

        # Client per il servizio is_motion_ready
        self.is_ready_client = self.create_client(
            IsMotionReady,
            '/play_motion2/is_motion_ready'
        )

        self.get_logger().info("PlayMotionClient pronto per usare i servizi.")

    # ==========================================================
    # FUNZIONE PER LISTARE LE MOTION
    # ==========================================================
    def list_motions(self):
        while not self.list_motions_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("In attesa del servizio /list_motions...")
        req = ListMotions.Request()
        future = self.list_motions_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    # ==========================================================
    # FUNZIONE PER VERIFICARE SE UNA MOTION È READY
    # ==========================================================
    def is_motion_ready(self, motion_name: str):
        while not self.is_ready_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("In attesa del servizio /is_motion_ready...")
        req = IsMotionReady.Request()
        req.motion_key = motion_name
        future = self.is_ready_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    # ==========================================================
    # FUNZIONE PER ESEGUIRE MOTION CON ROS2 RUN
    # ==========================================================
    def run_motion(self, motion_name: str):
        self.get_logger().info(f"Esecuzione motion: {motion_name}")
        # subprocess.run esegue il comando ROS2 run come se fosse da terminale
        result = subprocess.run(
            ['ros2', 'run', 'play_motion2', 'run_motion', motion_name],
            capture_output=True, text=True
        )
        self.get_logger().info(result.stdout)
        if result.returncode == 0:
            self.get_logger().info(f"Motion {motion_name} completata con successo.")
            return True
        else:
            self.get_logger().error(f"Errore nell'eseguire motion {motion_name}:\n{result.stderr}")
            return False


# ==========================================================
# MAIN
# ==========================================================
def main(args=None):
    rclpy.init(args=args)

    client = PlayMotionClient()

    # -----------------------------
    # 1) Lista tutte le motion
    # -----------------------------
    motions = client.list_motions()
    print("Motions disponibili:", motions.motion_keys)

    # -----------------------------
    # 2) Controlla se 'head_tour' è pronta
    # -----------------------------
    ready = client.is_motion_ready('head_tour')
    print("Head_tour pronta?", ready.is_ready)

    # -----------------------------
    # 3) Esegui head_tour
    # -----------------------------
    client.run_motion('head_tour')

    # -----------------------------
    # 4) Esegui unfold_arm
    # -----------------------------
    client.run_motion('unfold_arm')

    # Pulizia
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()