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
    """Z축 회전(yaw, radian)을 쿼터니언(x, y, z, w)으로 변환."""
    q = {}
    q['x'] = 0.0
    q['y'] = 0.0
    q['z'] = math.sin(yaw / 2.0)
    q['w'] = math.cos(yaw / 2.0)
    return q


def normalize_angle(angle):
    """각도를 -π ~ π 범위로 정규화"""
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

        # self.src_pts = np.array([
        #     [-0.96,  0.41],
        #     [-0.98, -0.49],
        #     [ 0.97,  0.39],
        #     [ 0.99, -0.53]
        # ], dtype=np.float32)
        #  src_pts 좌표도 40% 감소 (0.6배)
        self.src_pts = np.array([
            [-1.564,  0.090],  
            [-1.637, -0.847],  
            [ 0.382,  0.057],  
            [ 0.342, -0.854]
        ], dtype=np.float32)
        self.dst_pts = np.array([
            [0.05, 0.05],
            [0.05, 0.95],
            [1.95, 0.05],
            [1.95, 0.95]
        ], dtype=np.float32)
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
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                tvec = tvecs[i][0]
                rvec = rvecs[i][0]
                
                # 원본 3D 카메라 좌표 (맵핑 전)
                raw_x = round(float(tvec[0]), 3)
                raw_y = round(float(tvec[1]), 3)
                raw_z = round(float(tvec[2]), 3)
                
                # 맵핑된 2D 좌표 (기존 로직 유지)
                x, y = self.transform_point((tvec[0], tvec[1]))
                x, y = round(x, 2), round(y, 2)

                rot_mat, _ = cv2.Rodrigues(rvec)
                yaw = -(np.arctan2(rot_mat[1, 0], rot_mat[0, 0]) - np.pi / 2)
                yaw = normalize_angle(yaw)
                yaw = round(float(yaw), 3)

                corners_reshaped = corners[i].reshape((4, 2))
                center_x = int(np.mean(corners_reshaped[:, 0]))
                center_y = int(np.mean(corners_reshaped[:, 1]))

                detected.append({
                    "id": marker_id,
                    "robot_xy": [x, y],  # 맵핑된 좌표 (기존 로직용)
                    "raw_xyz": [raw_x, raw_y, raw_z],  # 원본 3D 카메라 좌표
                    "yaw": yaw,
                    "text_pos": (center_x, center_y)
                })
        return detected


class RobotState:
    """각 로봇의 상태를 저장하는 클래스"""
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0
        self.last_time = None
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_yaw = 0.0
        self.initialized = False


class ArucoOdometryPublisher(Node):
    def __init__(self, camera_matrix, dist_coeffs):
        super().__init__('aruco_odometry_publisher')
        self.detector = ArucoDetector(camera_matrix, dist_coeffs)
        self.cap = cv2.VideoCapture("/dev/video_cam", cv2.CAP_V4L2)  # V4L2 백엔드 명시적으로 지정
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        # 1. 해상도 설정 (40% 감소)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1200)   # 1920 * 0.6 = 1152
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)   # 1080 * 0.6 = 648
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # FPS 설정
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        # 설정이 제대로 되었는지 확인
        width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        print(f"Width: {width}, Height: {height}, FPS: {fps}")


        # Publishers
        self.json_pub = self.create_publisher(String, 'detected_markers', 10)
        self.path_publishers = {}    # id -> Path publisher
        self.pose_publishers = {}    # id -> PoseStamped publisher
        self.odom_publishers = {}    # id -> Odometry publisher
        
        # Data storage
        self.paths = {}              # id -> Path()
        self.robot_states = {}       # id -> RobotState()
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # 속도 계산을 위한 필터링 파라미터
        self.velocity_alpha = 0.7  # 저역 통과 필터 계수 (0~1, 1에 가까울수록 이전 값 유지)
        
        # Nav2 호환을 위한 frame 설정
        self.odom_frame = "odom"
        self.map_frame = "map"

    def update_robot_state(self, marker_id, x, y, yaw, current_time):
        """로봇 상태 업데이트 및 속도 계산"""
        if marker_id not in self.robot_states:
            self.robot_states[marker_id] = RobotState()

        state = self.robot_states[marker_id]
        
        if not state.initialized:
            # 첫 번째 측정값
            state.x = x
            state.y = y
            state.yaw = yaw
            state.last_x = x
            state.last_y = y
            state.last_yaw = yaw
            state.last_time = current_time
            state.initialized = True
            return

        # 시간 차이 계산
        dt = current_time - state.last_time
        if dt <= 0:
            return

        # 위치 변화량 계산
        dx = x - state.last_x
        dy = y - state.last_y
        dyaw = normalize_angle(yaw - state.last_yaw)

        # 속도 계산
        vx_new = dx / dt
        vy_new = dy / dt
        vyaw_new = dyaw / dt

        # 저역 통과 필터 적용 (노이즈 감소)
        state.vx = self.velocity_alpha * state.vx + (1 - self.velocity_alpha) * vx_new
        state.vy = self.velocity_alpha * state.vy + (1 - self.velocity_alpha) * vy_new
        state.vyaw = self.velocity_alpha * state.vyaw + (1 - self.velocity_alpha) * vyaw_new

        # 현재 상태 업데이트
        state.x = x
        state.y = y
        state.yaw = yaw
        state.last_x = x
        state.last_y = y
        state.last_yaw = yaw
        state.last_time = current_time

    def create_odometry_message(self, marker_id, current_time):
        """Nav2 호환 Odometry 메시지 생성"""
        if marker_id not in self.robot_states:
            return None

        state = self.robot_states[marker_id]
        if not state.initialized:
            return None

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        # Nav2 표준: odom frame 사용
        odom.header.frame_id = "odom"
        # Nav2 표준: base_link 또는 base_footprint 사용
        odom.child_frame_id = f"base_footprint_{marker_id}"

        # Position (global coordinates)
        odom.pose.pose.position.x = state.x
        odom.pose.pose.position.y = state.y
        odom.pose.pose.position.z = 0.0

        # Orientation
        q = yaw_to_quaternion(state.yaw)
        odom.pose.pose.orientation.x = q['x']
        odom.pose.pose.orientation.y = q['y']
        odom.pose.pose.orientation.z = q['z']
        odom.pose.pose.orientation.w = q['w']

        # Velocity in body frame (로봇 좌표계)
        cos_yaw = math.cos(state.yaw)
        sin_yaw = math.sin(state.yaw)
        
        # Global velocity를 body frame으로 변환
        vx_body = cos_yaw * state.vx + sin_yaw * state.vy
        vy_body = -sin_yaw * state.vx + cos_yaw * state.vy
        
        odom.twist.twist.linear.x = vx_body
        odom.twist.twist.linear.y = vy_body
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = state.vyaw

        # Nav2 호환 Covariance matrices
        # Position covariance (x, y, z, roll, pitch, yaw)
        pose_cov = [
            0.01, 0.0,  0.0,    0.0,    0.0,    0.0,     # x
            0.0,  0.01, 0.0,    0.0,    0.0,    0.0,     # y  
            0.0,  0.0,  1e6,    0.0,    0.0,    0.0,     # z (unused)
            0.0,  0.0,  0.0,    1e6,    0.0,    0.0,     # roll (unused)
            0.0,  0.0,  0.0,    0.0,    1e6,    0.0,     # pitch (unused)
            0.0,  0.0,  0.0,    0.0,    0.0,    0.02     # yaw
        ]
        
        # Velocity covariance (vx, vy, vz, vroll, vpitch, vyaw)
        twist_cov = [
            0.02, 0.0,  0.0,    0.0,    0.0,    0.0,     # vx
            0.0,  0.02, 0.0,    0.0,    0.0,    0.0,     # vy
            0.0,  0.0,  1e6,    0.0,    0.0,    0.0,     # vz (unused)
            0.0,  0.0,  0.0,    1e6,    0.0,    0.0,     # vroll (unused)
            0.0,  0.0,  0.0,    0.0,    1e6,    0.0,     # vpitch (unused)
            0.0,  0.0,  0.0,    0.0,    0.0,    0.05     # vyaw
        ]

        odom.pose.covariance = pose_cov
        odom.twist.covariance = twist_cov

        return odom

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Camera frame not received!")
            return

        current_time = time.time()
        detected = self.detector.detect_markers(frame)
        
        # JSON publish
        self.json_pub.publish(String(data=json.dumps([
            {"id": d["id"], "robot_xy": d["robot_xy"], "yaw": d["yaw"]} for d in detected
        ])))

        for marker in detected:
            marker_id = marker["id"]
            x, y = marker["robot_xy"]  # 맵핑된 좌표 (기존 로직용)
            raw_x, raw_y, raw_z = marker["raw_xyz"]  # 원본 3D 카메라 좌표
            yaw = marker["yaw"]
            cx, cy = marker["text_pos"]

            # 로봇 상태 업데이트 (맵핑된 좌표 사용)
            self.update_robot_state(marker_id, x, y, yaw, current_time)

            # Publishers 생성 (처음 감지될 때)
            if marker_id not in self.path_publishers:
                # Nav2 표준 토픽명 사용
                self.path_publishers[marker_id] = self.create_publisher(
                    Path, f"/robot{marker_id}/path", 10)
                self.pose_publishers[marker_id] = self.create_publisher(
                    PoseStamped, f"/robot{marker_id}/pose", 10)
                # Nav2 표준: /odom 토픽 사용
                self.odom_publishers[marker_id] = self.create_publisher(
                    Odometry, f"/odom_{marker_id}", 10)
                
                path = Path()
                path.header.frame_id = self.map_frame
                self.paths[marker_id] = path

            # 화면 표시 - 원본 3D 카메라 좌표 표시
            cv2.putText(frame, f"ID: {marker_id}", (cx -150, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 128, 0), 2)
            cv2.putText(frame, f"Raw XYZ: ({raw_x:.3f}, {raw_y:.3f}, {raw_z:.3f})", (cx -150, cy + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 128, 0), 2)
            cv2.putText(frame, f"Yaw: {yaw:.3f}", (cx -150, cy + 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 128, 0), 2)
            
            # 속도 정보 표시 (맵핑된 좌표 기반)
            if marker_id in self.robot_states and self.robot_states[marker_id].initialized:
                state = self.robot_states[marker_id]
                speed = math.sqrt(state.vx**2 + state.vy**2)
                cv2.putText(frame, f"Speed: {speed:.2f} m/s", (cx -150, cy + 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, f"Ang vel: {state.vyaw:.2f} rad/s", (cx -150, cy + 80),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # PoseStamped 생성 및 publish (map frame 사용, 맵핑된 좌표)
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = self.map_frame
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            q = yaw_to_quaternion(yaw)
            pose.pose.orientation.x = q['x']
            pose.pose.orientation.y = q['y']
            pose.pose.orientation.z = q['z']
            pose.pose.orientation.w = q['w']

            # Path 누적 및 publish
            self.paths[marker_id].header.stamp = self.get_clock().now().to_msg()
            self.paths[marker_id].poses.append(pose)
            self.path_publishers[marker_id].publish(self.paths[marker_id])
            self.pose_publishers[marker_id].publish(pose)

            # Odometry publish
            odom = self.create_odometry_message(marker_id, current_time)
            if odom is not None:
                odom.header.frame_id = f"odom_{marker_id}"  # 로봇 별 odom 프레임
                odom.child_frame_id = f"base_link_{marker_id}"
                self.odom_publishers[marker_id].publish(odom)

            # Nav2 호환 TF 브로드캐스트
            # 1. map -> odom (static, 일반적으로 AMCL이 담당하지만 여기서는 단순화)
            # t_map_odom = TransformStamped()
            # t_map_odom.header.stamp = self.get_clock().now().to_msg()
            # t_map_odom.header.frame_id = self.map_frame
            # t_map_odom.child_frame_id = f"{self.odom_frame}_{marker_id}"
            # # Identity transform (map == odom으로 가정)
            # t_map_odom.transform.translation.x = 0.0
            # t_map_odom.transform.translation.y = 0.0
            # t_map_odom.transform.translation.z = 0.0
            # t_map_odom.transform.rotation.x = 0.0
            # t_map_odom.transform.rotation.y = 0.0
            # t_map_odom.transform.rotation.z = 0.0
            # t_map_odom.transform.rotation.w = 1.0
            # self.tf_broadcaster.sendTransform(t_map_odom)

            # 2. odom -> base_link (odometry data)
            # t_odom_base = TransformStamped()
            # t_odom_base.header.stamp = self.get_clock().now().to_msg()
            # t_odom_base.header.frame_id = f"{self.odom_frame}_{marker_id}"
            # t_odom_base.child_frame_id = f"base_link_{marker_id}"

            # t_odom_base.transform.translation.x = x
            # t_odom_base.transform.translation.y = y
            # t_odom_base.transform.translation.z = 0.0
            # t_odom_base.transform.rotation.x = q['x']
            # t_odom_base.transform.rotation.y = q['y']
            # t_odom_base.transform.rotation.z = q['z']
            # t_odom_base.transform.rotation.w = q['w']

            # self.tf_broadcaster.sendTransform(t_odom_base)

        cv2.imshow("Aruco Odometry Publisher", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    base_path = '/home/addinnedu/monitoring_camera_ws/src/aruco_marker_pkg/include/'
    camera_matrix = np.load(base_path + 'camera_matrix.npy')
    dist_coeffs = np.load(base_path + 'dist_coeffs.npy')

    node = ArucoOdometryPublisher(camera_matrix, dist_coeffs)
    try:
        while rclpy.ok():
            node.process_frame()
            rclpy.spin_once(node, timeout_sec=0.005)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()