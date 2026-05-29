#!/usr/bin/env python3

# ==========================================================
# CLIENT ACTION PER MOVEHEAD
# ==========================================================


import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# Import della tua action custom
from my_robot_interfaces.action import MoveHead


class MoveHeadClient(Node):
    def __init__(self):
        super().__init__('move_head_client')

        # Creiamo l'action client verso il server MoveHead
        self._action_client = ActionClient(self, MoveHead, '/move_head')

        self.get_logger().info("MoveHeadClient pronto per inviare goal.")

    # ------------------------------------------------------
    def send_goal(self, min_pos, max_pos, step):
        # Aspetta che il server sia pronto
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Attendo il server /move_head...")

        # Creiamo il messaggio goal
        goal_msg = MoveHead.Goal()
        goal_msg.min = min_pos
        goal_msg.max = max_pos
        goal_msg.step = step

        # Invia il goal in modo asincrono
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # Spin finché il goal non viene accettato
        rclpy.spin_until_future_complete(self, self._send_goal_future)
        self._goal_handle = self._send_goal_future.result()

        if not self._goal_handle.accepted:
            self.get_logger().error("Goal rifiutato dal server!")
            return

        self.get_logger().info("Goal accettato, inizio movimento...")

        # Aspetta il risultato finale
        self._get_result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self._get_result_future)
        result = self._get_result_future.result().result

        if result.success:
            self.get_logger().info("Movimento completato con successo!")
        else:
            self.get_logger().error("Movimento fallito!")

    # ------------------------------------------------------
    def feedback_callback(self, feedback_msg):
        # Questo viene chiamato ad ogni aggiornamento dal server
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback: current_position = {feedback.current_position:.3f}")


# ==========================================================
# MAIN
# ==========================================================
def main(args=None):
    rclpy.init(args=args)
    client = MoveHeadClient()

    # Invio goal: movimento oscillante da -0.217 a 0.217 con passo 0.05
    # Cambiato da Gab e Tom in -0.3, 0.3 e passo più veloce
    client.send_goal(-0.3, 0.3, 0.1)

    # Pulizia
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
