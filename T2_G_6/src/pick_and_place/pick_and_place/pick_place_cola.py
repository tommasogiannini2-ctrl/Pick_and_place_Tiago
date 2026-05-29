import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import PoseStamped
from gazebo_ros_link_attacher.srv import Attach
from roboticstoolbox import ERobot
from spatialmath import SE3
from scipy.spatial.transform import Rotation as R
import numpy as np
import time


class TiagoPickAndPlace(Node):
    def __init__(self):
        super().__init__("tiago_pick_place_cola_node")

        # ==================================================
        # Joint names
        # ==================================================
        self.joint_names = [
            'torso_lift_joint',
            'arm_1_joint',
            'arm_2_joint',
            'arm_3_joint',
            'arm_4_joint',
            'arm_5_joint',
            'arm_6_joint',
            'arm_7_joint'
        ]

        # ==================================================
        # Subscribers
        # ==================================================
        self.create_subscription(
            JointTrajectoryControllerState,
            '/arm_controller/controller_state',
            self.arm_callback,
            10
        )

        self.create_subscription(
            JointTrajectoryControllerState,
            '/torso_controller/controller_state',
            self.torso_callback,
            10
        )

        self.create_subscription(
            PoseStamped,
            '/aruco_base_2',
            self.pick_marker_callback,
            10
        )


        # ==================================================
        # Publishers
        # ==================================================
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        self.torso_pub = self.create_publisher(
            JointTrajectory,
            '/torso_controller/joint_trajectory',
            10
        )

        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10
        )

        # ==================================================
        # Robot
        # ==================================================
        urdf_loc = '/home/ubuntu/Progetto_6_ws/src/my_robot_description/urdf/tiago_robot.urdf'
        self.robot = ERobot.URDF(urdf_loc)

        self.T_pick = SE3.Ry(np.pi / 2) * SE3.Rz(-np.pi / 2)

        # ==================================================
        # State variables
        # ==================================================
        self.pick_pose = None

        self.pick_received = False

        self.q0_ready = False
        self.q0_arm = None
        self.q0_torso = None

        self.q_init_arm = None
        self.q_init_torso = None
        self.init_saved = False

        self.q_sent = None

        self.pick_done = False
        self.attached = False

        self.state = "WAIT_PICK"
        self.wait_until = None

        # ==================================================
        # Services
        # ==================================================
        self.attach_client = self.create_client(Attach, '/attach')
        self.detach_client = self.create_client(Attach, '/detach')

        while not self.attach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Attesa servizio /attach...")

        while not self.detach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Attesa servizio /detach...")

        # ==================================================
        # Timer
        # ==================================================
        self.timer = self.create_timer(0.5, self.loop)

        self.get_logger().info("Nodo Pick & Place Coca-Cola avviato")

    # ==================================================
    # CALLBACKS
    # ==================================================
    def arm_callback(self, msg):
        self.q0_arm = np.array(msg.actual.positions)
        self.check_q0_ready()

    def torso_callback(self, msg):
        self.q0_torso = np.array(msg.actual.positions)
        self.check_q0_ready()

    def check_q0_ready(self):
        self.q0_ready = (
            self.q0_arm is not None and
            self.q0_torso is not None
        )

        if self.q0_ready and not self.init_saved:
            self.q_init_arm = self.q0_arm.copy()
            self.q_init_torso = self.q0_torso.copy()
            self.init_saved = True

            self.get_logger().info("Configurazione iniziale salvata")

    def pick_marker_callback(self, msg):
        if not self.pick_received:
            self.pick_pose = msg
            self.pick_received = True
            self.get_logger().info("Pick marker ricevuto")


    # ==================================================
    # GRIPPER
    # ==================================================
    def open_gripper(self):
        traj = JointTrajectory()
        traj.joint_names = [
            'gripper_left_finger_joint',
            'gripper_right_finger_joint'
        ]

        point = JointTrajectoryPoint()
        point.positions = [0.44, 0.44]
        point.time_from_start.sec = 2

        traj.points.append(point)

        self.gripper_pub.publish(traj)

        self.get_logger().info("Gripper aperto")

    def close_gripper(self):
        traj = JointTrajectory()
        traj.joint_names = [
            'gripper_left_finger_joint',
            'gripper_right_finger_joint'
        ]

        point = JointTrajectoryPoint()
        point.positions = [0.41, 0.41]
        point.time_from_start.sec = 2

        traj.points.append(point)

        self.gripper_pub.publish(traj)

        self.get_logger().info("Gripper chiuso")

    # ==================================================
    # MAIN LOOP
    # ==================================================
    def loop(self):
        if not self.q0_ready:
            return

        if not hasattr(self, 'gripper_opened'):
            self.open_gripper()
            self.gripper_opened = True
            return

        # =========================
        # WAIT PICK
        # =========================
        if self.state == "WAIT_PICK":
            if self.pick_pose is None:
                return

            self.get_logger().info("Muovo verso PICK")

            pos_above = self._get_offset_position(
                self.pick_pose,
                dx=-0.0035,
                dy=-0.045,
                dz=-0.1
            )

            TF_above = self._make_SE3(
                self.pick_pose,
                pos_above
            ) * self.T_pick

            q_target = self.robot.ik_NR(
                TF_above,
                q0=self.q_sent,
                pinv=True
            )[0]

            self.send_trajectory(q_target)
            self.q_sent = q_target

            self.wait_until = time.time() + 4
            self.state = "LOWER_PICK"
            return

        # =========================
        # LOWER PICK
        # =========================
        if self.state == "LOWER_PICK":
            if time.time() < self.wait_until:
                return

            self.get_logger().info("Abbasso di 4 cm")

            pos_lower = self._get_offset_position(
                self.pick_pose,
                dx=-0.0035,
                dy=-0.045,
                dz=-0.14
            )

            TF_lower = self._make_SE3(
                self.pick_pose,
                pos_lower
            ) * self.T_pick

            q_lower = self.robot.ik_NR(
                TF_lower,
                q0=self.q_sent,
                pinv=True
            )[0]

            self.send_trajectory(q_lower)
            self.q_sent = q_lower

            self.wait_until = time.time() + 2
            self.state = "ATTACH"
            return

        # =========================
        # ATTACH
        # =========================
        if self.state == "ATTACH":
            if time.time() < self.wait_until:
                return
            if not hasattr(self, 'gripper_closed'):
                self.close_gripper()
                self.gripper_closed = True
                return
            self.attach_object()

            self.attached = True
            self.pick_done = True

            self.get_logger().info("Oggetto preso!")

            self.state = "MOVE_PLACE"
            return

        

    # ==================================================
    # UTILITIES
    # ==================================================
    def _get_offset_position(self, pose, dx=0, dy=0, dz=0):
        return np.array([
            pose.pose.position.x + dx,
            pose.pose.position.y + dy,
            pose.pose.position.z + dz
        ])

    def _make_SE3(self, pose, position):
        quat = [
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
        ]

        R_mat = R.from_quat(quat).as_matrix()

        return SE3.Rt(R_mat, position)

    def send_trajectory(self, q):
        q = np.array(q).flatten()

        traj_arm = JointTrajectory()
        traj_torso = JointTrajectory()

        traj_arm.joint_names = self.joint_names[1:]
        traj_torso.joint_names = [self.joint_names[0]]

        point_arm = JointTrajectoryPoint()
        point_torso = JointTrajectoryPoint()

        point_arm.positions = [float(x) for x in q[1:]]
        point_torso.positions = [float(q[0])]

        point_arm.time_from_start.sec = 3
        point_torso.time_from_start.sec = 3

        traj_arm.points.append(point_arm)
        traj_torso.points.append(point_torso)

        self.arm_pub.publish(traj_arm)
        self.torso_pub.publish(traj_torso)

    def attach_object(self):
        req = Attach.Request()
        req.model_name_1 = 'tiago'
        req.link_name_1 = 'gripper_left_finger_link'
        req.model_name_2 = 'cocacola'
        req.link_name_2 = 'link'

        self.attach_client.call_async(req)

        self.get_logger().info("Richiesta attach inviata")

    def detach_object(self):
        req = Attach.Request()
        req.model_name_1 = 'tiago'
        req.link_name_1 = 'gripper_left_finger_link'
        req.model_name_2 = 'cocacola'
        req.link_name_2 = 'link'

        self.detach_client.call_async(req)

        self.get_logger().info("Richiesta detach inviata")


def main(args=None):
    rclpy.init(args=args)

    node = TiagoPickAndPlace()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
