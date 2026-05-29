import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped

import cv2
import numpy as np
from cv_bridge import CvBridge

import tf2_ros
from scipy.spatial.transform import Rotation as R


class ArucoDetector(Node):

    def __init__(self):
        super().__init__('aruco_detector_simple')

        # Subscriber
        self.create_subscription(
            Image,
            '/head_front_camera/rgb/image_raw',
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/head_front_camera/rgb/camera_info',
            self.camera_info_callback,
            10
        )

        # Timer republish
        self.timer = self.create_timer(0.1, self.republish)

        # Variabili
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        # Usiamo dizionari per supportare più marker contemporaneamente
        self.last_poses_camera = {}
        self.last_poses_base = {}

        # Publisher dinamici (UNO per ID)
        self.camera_publishers = {}
        self.base_publishers = {}

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.base_frame = 'base_footprint'
        self.camera_frame = 'head_front_camera_color_optical_frame'

        # ArUco
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.params = cv2.aruco.DetectorParameters_create()
        self.marker_size = 0.06

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        self.destroy_subscription(self.camera_info_sub)

    def image_callback(self, msg):

        if self.camera_matrix is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.params
        )

        if ids is None:
            cv2.imshow("Aruco", frame)
            cv2.waitKey(1)
            return
            
        # Disegniamo i bordi di tutti i marker trovati (fuori dal ciclo)
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # CICLO: ora tutto il blocco è indentato correttamente
        for i in range(len(ids)):
            marker_id = int(ids[i][0])

            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners[i],
                self.marker_size,
                self.camera_matrix,
                self.dist_coeffs
            )

            # Disegniamo l'asse per il marker corrente (usiamo rvec[0], tvec[0])
            cv2.aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs, rvec[0], tvec[0], 0.03)

            # ==========================
            # CREATE PUBLISHER DINAMICI
            # ==========================
            if marker_id not in self.camera_publishers:
                topic_cam = f"/aruco_camera_{marker_id}"
                self.camera_publishers[marker_id] = self.create_publisher(PoseStamped, topic_cam, 10)

            if marker_id not in self.base_publishers:
                topic_base = f"/aruco_base_{marker_id}"
                self.base_publishers[marker_id] = self.create_publisher(PoseStamped, topic_base, 10)

            # ==========================
            # POSE CAMERA
            # ==========================
            pose_cam = PoseStamped()
            pose_cam.header.stamp = msg.header.stamp
            pose_cam.header.frame_id = self.camera_frame

            pose_cam.pose.position.x = float(tvec[0][0][0])
            pose_cam.pose.position.y = float(tvec[0][0][1])
            pose_cam.pose.position.z = float(tvec[0][0][2])

            # Calcolo quaternione dalla rotazione rvec
            quat = R.from_rotvec(rvec[0][0]).as_quat()
            pose_cam.pose.orientation.x = float(quat[0])
            pose_cam.pose.orientation.y = float(quat[1])
            pose_cam.pose.orientation.z = float(quat[2])
            pose_cam.pose.orientation.w = float(quat[3])

            # Salviamo nel dizionario per il republish
            self.last_poses_camera[marker_id] = pose_cam

            self.camera_publishers[marker_id].publish(pose_cam)

            # ==========================
            # POSE BASE
            # ==========================
            if self.tf_buffer.can_transform(
                self.base_frame,
                self.camera_frame,
                Time(),
                timeout=Duration(seconds=0.1)
            ):
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.base_frame,
                        self.camera_frame,
                        Time()
                    )

                    pos_cam = np.array([tvec[0][0][0], tvec[0][0][1], tvec[0][0][2]])

                    q = transform.transform.rotation
                    r = R.from_quat([q.x, q.y, q.z, q.w])

                    pos_base = r.as_matrix() @ pos_cam
                    pos_base += np.array([
                        transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z
                    ])

                    r_marker = R.from_rotvec(rvec[0][0])
                    r_base = r * r_marker
                    quat_base = r_base.as_quat()

                    pose_base = PoseStamped()
                    pose_base.header.stamp = msg.header.stamp
                    pose_base.header.frame_id = self.base_frame

                    pose_base.pose.position.x = float(pos_base[0])
                    pose_base.pose.position.y = float(pos_base[1])
                    pose_base.pose.position.z = float(pos_base[2])

                    pose_base.pose.orientation.x = float(quat_base[0])
                    pose_base.pose.orientation.y = float(quat_base[1])
                    pose_base.pose.orientation.z = float(quat_base[2])
                    pose_base.pose.orientation.w = float(quat_base[3])

                    # Salviamo nel dizionario per il republish
                    self.last_poses_base[marker_id] = pose_base

                    self.base_publishers[marker_id].publish(pose_base)
                except Exception as e:
                    self.get_logger().error(f"Errore TF: {e}")

        cv2.imshow("Aruco", frame)
        cv2.waitKey(1)

    # ==========================
    # REPUBLISH
    # ==========================
    def republish(self):
        # Cicliamo su tutti i marker salvati nei dizionari
        for m_id in self.last_poses_camera:
            self.camera_publishers[m_id].publish(self.last_poses_camera[m_id])
            
        for m_id in self.last_poses_base:
            self.base_publishers[m_id].publish(self.last_poses_base[m_id])


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
