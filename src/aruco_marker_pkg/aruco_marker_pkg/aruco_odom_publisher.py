#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, Odometry
import json
import math
import time
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


def yaw_to_quaternion(yaw):
    q = {}
    q['x'] = 0.0
    q['y'] = 0.0
    q['z'] = math.sin(yaw / 2.0)
    q['w'] = math.cos(yaw / 2.0)
    return q


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class ArucoDetector:
    def __init__(self, camera_matrix, dist_coeffs, marker_length=0.05):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.marker_length = marker_length
        self.src_pts = np.array([[-0.96,  0.41], [-0.98, -0.49], [0.97,  0.39], [0.99, -0.53]], dtype=np.float32)
        self.dst_pts = np.array([[0.05, 0.05], [0.05, 0.95], [1.95, 0.05], [1.95, 0.95]], dtype=np.float32)
        self.H = cv2.getPerspectiveTransform(self.src_pts, self.dst_pts)

    def transform_point(self, pt):
        src = np.array([pt[0], pt[1], 1.0], dtype=np.float32)
        dst = self.H @ src
        dst = dst / dst[2]
        return float(dst[0]), float(dst[1])

    def detect_markers(self, frame):
        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)
        detected = []
        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
            for i in range(len(ids)):
                tvec = tvecs[i][0]
                rvec = rvecs[i][0]
                x, y = self.transform_point((tvec[0], tvec[1]))
                x, y = round(x, 2), round(y, 2)

                rot_mat, _ = cv2.Rodrigues(rvec)
                #yaw = np.arctan2(rot_mat[1, 0], rot_mat[0, 0]) - np.pi / 2
                yaw = -(np.arctan2(rot_mat[1, 0], rot_mat[0, 0]) - np.pi / 2)
                yaw = normalize_angle(yaw)
                yaw = round(float(yaw), 3)

                corners_reshaped = corners[i].reshape((4, 2))
                center_x = int(np.mean(corners_reshaped[:, 0]))
                center_y = int(np.mean(corners_reshaped[:, 1]))

                detected.append({
                    "robot_xy": [x, y],
                    "yaw": yaw,
                    "text_pos": (center_x, center_y)
                })
        return detected


class RobotState:
    def __init__(self):
        self.x = self.y = self.yaw = self.vx = self.vy = self.vyaw = 0.0
        self.last_time = self.last_x = self.last_y = self.last_yaw = 0.0
        self.initialized = False


class ArucoOdometryPublisher(Node):
    def __init__(self, camera_matrix, dist_coeffs):
        super().__init__('aruco_odometry_publisher')
        self.detector = ArucoDetector(camera_matrix, dist_coeffs)
        self.cap = cv2.VideoCapture("/dev/video_cam")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        self.json_pub = self.create_publisher(String, 'detected_markers', 10)
        self.path_pub = self.create_publisher(Path, "/path", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/pose", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

        self.path = Path()
        self.path.header.frame_id = "map"
        self.robot_state = RobotState()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.velocity_alpha = 0.7

    def update_robot_state(self, x, y, yaw, current_time):
        state = self.robot_state
        if not state.initialized:
            state.x = x
            state.y = y
            state.yaw = yaw
            state.last_x = x
            state.last_y = y
            state.last_yaw = yaw
            state.last_time = current_time
            state.initialized = True
            return

        dt = current_time - state.last_time
        if dt <= 0: return

        dx = x - state.last_x
        dy = y - state.last_y
        dyaw = normalize_angle(yaw - state.last_yaw)
        vx_new, vy_new, vyaw_new = dx / dt, dy / dt, dyaw / dt

        state.vx = self.velocity_alpha * state.vx + (1 - self.velocity_alpha) * vx_new
        state.vy = self.velocity_alpha * state.vy + (1 - self.velocity_alpha) * vy_new
        state.vyaw = self.velocity_alpha * state.vyaw + (1 - self.velocity_alpha) * vyaw_new

        state.x, state.y, state.yaw = x, y, yaw
        state.last_x, state.last_y, state.last_yaw = x, y, yaw
        state.last_time = current_time

    def create_odometry_message(self, current_time):
        state = self.robot_state
        if not state.initialized:
            return None
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = state.x
        odom.pose.pose.position.y = state.y
        q = yaw_to_quaternion(state.yaw)
        odom.pose.pose.orientation.x = q['x']
        odom.pose.pose.orientation.y = q['y']
        odom.pose.pose.orientation.z = q['z']
        odom.pose.pose.orientation.w = q['w']

        cos_yaw = math.cos(state.yaw)
        sin_yaw = math.sin(state.yaw)
        vx_body = cos_yaw * state.vx + sin_yaw * state.vy
        vy_body = -sin_yaw * state.vx + cos_yaw * state.vy
        odom.twist.twist.linear.x = vx_body
        odom.twist.twist.linear.y = vy_body
        odom.twist.twist.angular.z = state.vyaw
        return odom

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Camera frame not received!")
            return
        current_time = time.time()
        detected = self.detector.detect_markers(frame)
        if not detected: return

        marker = detected[0]
        x, y, yaw = marker["robot_xy"][0], marker["robot_xy"][1], marker["yaw"]
        self.update_robot_state(x, y, yaw, current_time)

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        q = yaw_to_quaternion(yaw)
        pose.pose.orientation.x = q['x']
        pose.pose.orientation.y = q['y']
        pose.pose.orientation.z = q['z']
        pose.pose.orientation.w = q['w']

        self.path.header.stamp = pose.header.stamp
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)
        self.pose_pub.publish(pose)

        odom = self.create_odometry_message(current_time)
        if odom:
            self.odom_pub.publish(odom)

        t_map_odom = TransformStamped()
        t_map_odom.header.stamp = pose.header.stamp
        t_map_odom.header.frame_id = "map"
        t_map_odom.child_frame_id = "odom"
        t_map_odom.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_map_odom)

        t_odom_base = TransformStamped()
        t_odom_base.header.stamp = pose.header.stamp
        t_odom_base.header.frame_id = "odom"
        t_odom_base.child_frame_id = "base_link"
        t_odom_base.transform.translation.x = x
        t_odom_base.transform.translation.y = y
        t_odom_base.transform.rotation.x = q['x']
        t_odom_base.transform.rotation.y = q['y']
        t_odom_base.transform.rotation.z = q['z']
        t_odom_base.transform.rotation.w = q['w']
        self.tf_broadcaster.sendTransform(t_odom_base)

        cv2.imshow("Aruco Odometry Publisher", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    import os
    from pathlib import Path
    
    # 현재 작업 디렉토리에서 워크스페이스 루트 찾기
    current_path = Path.cwd()
    
    # 워크스페이스 루트 찾기 (build나 install이 있는 디렉토리)
    workspace_root = current_path
    while workspace_root.name not in ['ROS2_Jazzy_Study'] and workspace_root.parent != workspace_root:
        workspace_root = workspace_root.parent
        
    # src 디렉토리의 패키지 경로
    base_path = workspace_root / 'src' / 'aruco_marker_pkg' / 'include'

    # 파일 존재 확인
    camera_matrix_path = base_path / 'camera_matrix.npy'
    dist_coeffs_path = base_path / 'dist_coeffs.npy'
    
    if not camera_matrix_path.exists():
        raise FileNotFoundError(f"camera_matrix.npy not found at {camera_matrix_path}")
    if not dist_coeffs_path.exists():
        raise FileNotFoundError(f"dist_coeffs.npy not found at {dist_coeffs_path}")
        
    camera_matrix = np.load(str(camera_matrix_path))
    dist_coeffs = np.load(str(dist_coeffs_path))

    node = ArucoOdometryPublisher(camera_matrix, dist_coeffs)
    try:
        while rclpy.ok():
            node.process_frame()
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
