#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient

from my_robot_interfaces.action import MoveHead

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class HeadActionServer(Node):

    def __init__(self):
        super().__init__('head_action_server')

        # Action server
        self._action_server = ActionServer(
            self,
            MoveHead,
            'move_head',
            self.execute_callback
        )

        # Client verso TIAGO
        self.follow_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/head_controller/follow_joint_trajectory'
        )

        self.joint_names = ['head_1_joint', 'head_2_joint']
        self.head2_fixed = -0.6

        self.get_logger().info("Action server custom pronto.")

    # ------------------------------------------------------
    def move_head(self, pos1):
        while not self.follow_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Attendo controller...")

        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [pos1, self.head2_fixed]
        point.time_from_start.sec = 1

        traj.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        future = self.follow_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        return True

    # ------------------------------------------------------
    def execute_callback(self, goal_handle):
        self.get_logger().info("Goal ricevuto!")

        feedback = MoveHead.Feedback()
        result = MoveHead.Result()

        current = goal_handle.request.max
        direction = -1

        while current >= goal_handle.request.min:

            # Movimento testa
            self.move_head(current)

            # Feedback reale
            feedback.current_position = current
            goal_handle.publish_feedback(feedback)

            # Aggiornamento
            current += goal_handle.request.step * direction

            if current <= goal_handle.request.min:
                direction = 1
            elif current >= goal_handle.request.max:
                direction = -1

        goal_handle.succeed()
        result.success = True

        return result


def main(args=None):
    rclpy.init(args=args)
    node = HeadActionServer()
    
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
