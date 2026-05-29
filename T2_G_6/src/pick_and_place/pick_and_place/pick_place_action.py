import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient 

from control_msgs.action import FollowJointTrajectory 
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from gazebo_ros_link_attacher.srv import Attach

from roboticstoolbox import ERobot
from spatialmath import SE3
from scipy.spatial.transform import Rotation as R
import numpy as np

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
        self.create_subscription(JointTrajectoryControllerState, '/arm_controller/controller_state', self.arm_callback, 10)
        self.create_subscription(JointTrajectoryControllerState, '/torso_controller/controller_state', self.torso_callback, 10)
        self.create_subscription(PoseStamped, '/aruco_base_2', self.pick_marker_callback, 10)
        self.create_subscription(PoseStamped, '/aruco_base_3', self.place_marker_callback, 10)

        # ==================================================
        # Action Clients
        # ==================================================
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.torso_client = ActionClient(self, FollowJointTrajectory, '/torso_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')

        self.get_logger().info("Attesa degli Action Server...")
        self.arm_client.wait_for_server()
        self.torso_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.get_logger().info("Action Server trovati!")

        # ==================================================
        # Robot Kinematics
        # ==================================================
        urdf_loc = '/home/ubuntu/Progetto_6_ws/src/my_robot_description/urdf/tiago_robot.urdf'
        self.robot = ERobot.URDF(urdf_loc)
        self.T_pick = SE3.Ry(np.pi / 2) * SE3.Rz(-np.pi / 2)
        self.T_place = SE3.Rx(np.pi / 2) * SE3.Rz( np.pi)
        

        # ==================================================
        # State variables
        # ==================================================
        self.pick_pose = None
        self.pick_received = False
        
        self.place_pose = None
        self.place_received = False

        self.q0_ready = False
        self.q0_arm = None
        self.q0_torso = None

        self.q_init_arm = None
        self.q_init_torso = None
        self.init_saved = False
        self.q_sent = None

        self.pick_done = False
        self.attached = False
        
        self.q_torso = None

        self.state = "INIT" 

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
        self.timer = self.create_timer(0.1, self.loop) 
        self.get_logger().info("Nodo Pick & Place Coca-Cola avviato con Action")

    # ==================================================
    # CALLBACKS SENSORI
    # ==================================================
    def arm_callback(self, msg):
        self.q0_arm = np.array(msg.actual.positions)
        self.check_q0_ready()

    def torso_callback(self, msg):
        self.q0_torso = np.array(msg.actual.positions)
        self.check_q0_ready()

    def check_q0_ready(self):
        self.q0_ready = (self.q0_arm is not None and self.q0_torso is not None)
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
            
    def place_marker_callback(self, msg):
        if not self.place_received:
            self.place_pose = msg
            self.place_received = True
            self.get_logger().info("Place marker (/aruco_base_3) ricevuto")


    def on_place_above_reached(self, future):
        self.get_logger().info("Arrivato sopra il punto di PLACE.")
        self.state = "RELEASE"


    def on_gripper_opened_place(self, future):
        self.get_logger().info("Gripper aperto. Oggetto posato.")
        self.attached = False
        self.state = "RETREAT"
        
    def on_retreat_reached(self, future):
        self.get_logger().info("Braccio ritratto. Pick & Place completato con successo!")
        self.state = "DONE" # Fine del programma!

    # ==================================================
    # GESTIONE ACTION CLIENT
    # ==================================================
    def send_action_goal(self, client, joint_names, positions, duration_sec, on_complete_callback=None):
        """Metodo di supporto per inviare le action in modo pulito"""
        goal_msg = FollowJointTrajectory.Goal()
        
        traj = JointTrajectory()
        traj.joint_names = joint_names
        
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start.sec = duration_sec
        traj.points.append(point)
        
        goal_msg.trajectory = traj

        # Invia il goal in modo asincrono
        send_goal_future = client.send_goal_async(goal_msg)
        
        if on_complete_callback:
            send_goal_future.add_done_callback(
                lambda future: self._goal_response_callback(future, on_complete_callback)
            )

    def _goal_response_callback(self, future, on_complete_callback):
        """Viene chiamata quando il server accetta o rifiuta il goal"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Il movimento è stato rifiutato dal controller!')
            return
        
        # Ora chiediamo di essere avvisati quando il movimento FISICO è concluso
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(on_complete_callback)

    # ==================================================
    # COMANDI MACRO E CALLBACK DI FINE MOVIMENTO
    # ==================================================
    def send_arm_torso_goal(self, q, callback):
        q = np.array(q).flatten()
        arm_positions = q[1:]
        torso_positions = [q[0]]

        # Muoviamo il torso
        self.send_action_goal(self.torso_client, [self.joint_names[0]], torso_positions, 3)
        # Muoviamo il braccio e diamo il via alla callback quando ha finito
        self.send_action_goal(self.arm_client, self.joint_names[1:], arm_positions, 3, callback)

    def open_gripper(self, callback):
        self.get_logger().info("Apertura gripper in corso...")
        self.send_action_goal(self.gripper_client, ['gripper_left_finger_joint', 'gripper_right_finger_joint'], [0.44, 0.44], 2, callback)

    def close_gripper(self, callback):
        self.get_logger().info("Chiusura gripper in corso...")
        self.send_action_goal(self.gripper_client, ['gripper_left_finger_joint', 'gripper_right_finger_joint'], [0.41, 0.41], 2, callback)

    def on_gripper_opened(self, future):
        self.get_logger().info("Gripper aperto con successo. Attesa del marker...")
        self.state = "WAIT_PICK"

    def on_above_reached(self, future):
        self.get_logger().info("Posizione di pre-pick raggiunta.")
        self.state = "LOWER_PICK"

    def on_lower_reached(self, future):
        self.get_logger().info("Sceso sull'oggetto.")
        self.state = "ATTACH"

    def on_gripper_closed(self, future):
        self.get_logger().info("Gripper chiuso sull'oggetto.")
        self.attach_object()
        self.attached = True
        self.pick_done = True
        self.state = "LIFT"
        self.get_logger().info("Pronto per andare verso LIFT.")
        
    def on_lift_reached(self, future):
        self.get_logger().info("Oggetto sollevato con successo di 10 cm.")
        self.state = "MOVE_PLACE"
        self.get_logger().info("Pronto per andare verso PLACE.")

    # ==================================================
    # MAIN LOOP E MACCHINA A STATI
    # ==================================================
    def loop(self):
        if not self.q0_ready:
            return

        # Lo stato "WAITING" blocca il loop per non spammmare chiamate. 
        # Lo stato verrà sbloccato dalle callback (on_gripper_opened, on_above_reached, ecc.)

        if self.state == "INIT":
            self.state = "WAITING"
            self.open_gripper(callback=self.on_gripper_opened)

        elif self.state == "WAIT_PICK":
            if self.pick_pose is None:
                return
            
            self.state = "WAITING"
            self.get_logger().info("Calcolo la traiettoria verso PICK (above)...")

            pos_above = self._get_offset_position(self.pick_pose, dx=-0.0035, dy=-0.045, dz=-0.1)
            TF_above = self._make_SE3(self.pick_pose, pos_above) * self.T_pick
            
            q_target = self.robot.ik_NR(TF_above, q0=self.q_sent, pinv=True)[0]
            self.q_sent = q_target
            
            self.send_arm_torso_goal(q_target, callback=self.on_above_reached)

        elif self.state == "LOWER_PICK":
            self.state = "WAITING"
            self.get_logger().info("Abbasso di 4 cm...")

            pos_lower = self._get_offset_position(self.pick_pose, dx=-0.0035, dy=-0.045, dz=-0.2) 
            TF_lower = self._make_SE3(self.pick_pose, pos_lower) * self.T_pick
            
            q_lower = self.robot.ik_NR(TF_lower, q0=self.q_sent, pinv=True)[0]
            self.q_sent = q_lower
            
            self.q_torso = q_lower[0]
            
            self.send_arm_torso_goal(q_lower, callback=self.on_lower_reached)

        elif self.state == "ATTACH":
            self.state = "WAITING"
            self.close_gripper(callback=self.on_gripper_closed)
            
            
        elif self.state == "LIFT":
            self.state = "WAITING"
            self.get_logger().info("Sollevo l'oggetto di 10 cm...")

            pos_lift = self._get_offset_position(
                self.pick_pose, 
                dx=-0.0035, 
                dy=-0.045, 
                dz=-0.04 
            )
            
            TF_lift = self._make_SE3(self.pick_pose, pos_lift) * self.T_pick
            
            q_lift = self.robot.ik_NR(TF_lift, q0=self.q_sent, pinv=True)[0]
            self.q_sent = q_lift
            
            self.send_arm_torso_goal(q_lift, callback=self.on_lift_reached)
            
        ############################PLACE
            
        elif self.state == "MOVE_PLACE":
            if self.place_pose is None:
                self.get_logger().info("In attesa di vedere il marker /aruco_base_4...", throttle_duration_sec=2.0)
                return 

            self.state = "WAITING"
            self.get_logger().info("Muovo verso l'alto sopra la posizione di PLACE...")

            pos_place_above = self._get_offset_position(self.place_pose, dx=0, dy=0, dz=-0.015) 
            TF_place_above = self._make_SE3(self.place_pose, pos_place_above) * self.T_place

            # Calcolo della cinematica inversa
            ik_solution = self.robot.ik_NR(TF_place_above, q0=self.q_sent, pinv=True)
            q_target = ik_solution[0]
            success = ik_solution[1] 

            if not success:
                self.get_logger().warn("ATTENZIONE: Cinematica fallita! L'orientamento potrebbe essere fuori portata.")

            tolerance = np.deg2rad(5.0)
            two_pi = 2 * np.pi

            for i in range(len(q_target)):
                diff = q_target[i] - self.q_sent[i]

                # Controlla se la differenza è circa +2pi o -2pi
                if abs(diff - two_pi) < tolerance or abs(diff + two_pi) < tolerance:
                    self.get_logger().info(f"Rilevato salto di 2pi sul giunto {i}, correggo...")
                    # Sovrascriviamo il valore del target con quello precedente
                    q_target[i] = self.q_sent[i]


            # Aggiorna lo stato dei giunti inviati e spedisci il comando
            self.q_sent = q_target
            self.send_arm_torso_goal(q_target, callback=self.on_place_above_reached)
            

        elif self.state == "RELEASE":
            self.state = "WAITING"
            self.get_logger().info("Scollego l'oggetto in Gazebo e apro il gripper...")
            
            self.detach_object() 
         
            self.open_gripper(callback=self.on_gripper_opened_place)

        elif self.state == "RETREAT":
            self.state = "WAITING"
            self.get_logger().info("Mi allontano dall'oggetto verso l'alto...")

            pos_retreat = self._get_offset_position(self.place_pose, dx=0, dy=0, dz=0.2)
            TF_retreat = self._make_SE3(self.place_pose, pos_retreat) * self.T_place
            
            q_retreat = self.robot.ik_NR(TF_retreat, q0=self.q_sent, pinv=True)[0]
            self.q_sent = q_retreat
            
            self.send_arm_torso_goal(q_retreat, callback=self.on_retreat_reached)
            
        elif self.state == "DONE":
            raise SystemExit           



    # ==================================================
    # UTILITIES / SERVICE
    # ==================================================
    def _get_offset_position(self, pose, dx=0, dy=0, dz=0):
        return np.array([
            pose.pose.position.x + dx,
            pose.pose.position.y + dy,
            pose.pose.position.z + dz
        ])

    def _make_SE3(self, pose, position):
        quat = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
        R_mat = R.from_quat(quat).as_matrix()
        return SE3.Rt(R_mat, position)

    def attach_object(self):
        req = Attach.Request()
        req.model_name_1, req.link_name_1 = 'tiago', 'gripper_left_finger_link'
        req.model_name_2, req.link_name_2 = 'cocacola', 'link'
        self.attach_client.call_async(req)

    def detach_object(self):
        req = Attach.Request()
        req.model_name_1, req.link_name_1 = 'tiago', 'gripper_left_finger_link'
        req.model_name_2, req.link_name_2 = 'cocacola', 'link'
        self.detach_client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = TiagoPickAndPlace()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
