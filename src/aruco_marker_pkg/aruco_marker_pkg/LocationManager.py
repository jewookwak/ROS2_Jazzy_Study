#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
import math
import time

# 커스텀 메시지 import
from aruco_interfaces.msg import ArucoPose, ArucoPoseArray

from std_msgs.msg import Header


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

        self.src_pts = np.array([
            [-0.698,  0.046],  
            [-0.739, -0.399],  
            [ 0.215,  0.030],  
            [ 0.202, -0.409]
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
                    "robot_xy": [x, y],
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
        self.cap = cv2.VideoCapture("/dev/video_cam", cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1200)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        print(f"Width: {width}, Height: {height}, FPS: {fps}")

        # 커스텀 메시지 Publisher 생성
        self.aruco_poses_array_publisher = self.create_publisher(ArucoPoseArray, '/aruco_poses', 10)
        
        # Data storage
        self.robot_states = {}
        
        # Low pass filter 계수들
        self.velocity_alpha = 0.3    # 속도용 필터 계수
        self.position_alpha = 0.7    # 위치용 필터 계수 (위치는 속도보다 덜 민감하게)
        self.yaw_alpha = 0.6         # yaw용 필터 계수
        
        self.map_frame = "map"

    def update_robot_state(self, marker_id, x_raw, y_raw, yaw_raw, current_time):
        """로봇 상태 업데이트 및 속도 계산 (위치와 yaw에도 low pass filter 적용)"""
        if marker_id not in self.robot_states:
            self.robot_states[marker_id] = RobotState()

        state = self.robot_states[marker_id]
        
        if not state.initialized:
            # 초기화 시에는 필터링 없이 바로 설정
            state.x = x_raw
            state.y = y_raw
            state.yaw = yaw_raw
            state.last_x = x_raw
            state.last_y = y_raw
            state.last_yaw = yaw_raw
            state.last_time = current_time
            state.initialized = True
            return

        dt = current_time - state.last_time
        if dt <= 0:
            return

        # Low pass filter 적용
        # 위치 필터링
        x_filtered = self.position_alpha * state.x + (1 - self.position_alpha) * x_raw
        y_filtered = self.position_alpha * state.y + (1 - self.position_alpha) * y_raw
        
        # Yaw 필터링 (각도 차이를 정규화한 후 필터링)
        yaw_diff = normalize_angle(yaw_raw - state.yaw)
        yaw_filtered = normalize_angle(state.yaw + (1 - self.yaw_alpha) * yaw_diff)

        # 속도 계산 (필터링된 위치를 기반으로)
        dx = x_filtered - state.last_x
        dy = y_filtered - state.last_y
        dyaw = normalize_angle(yaw_filtered - state.last_yaw)

        vx_new = dx / dt
        vy_new = dy / dt
        vyaw_new = dyaw / dt

        # 속도 필터링
        state.vx = self.velocity_alpha * state.vx + (1 - self.velocity_alpha) * vx_new
        state.vy = self.velocity_alpha * state.vy + (1 - self.velocity_alpha) * vy_new
        state.vyaw = self.velocity_alpha * state.vyaw + (1 - self.velocity_alpha) * vyaw_new

        # 상태 업데이트
        state.last_x = state.x
        state.last_y = state.y
        state.last_yaw = state.yaw
        
        state.x = x_filtered
        state.y = y_filtered
        state.yaw = yaw_filtered
        state.last_time = current_time

    def create_aruco_poses_array_message(self, detected_markers):
        """여러 ArUco 마커들의 정보를 담은 ArucoPoseArray 메시지 생성"""
        poses_array = ArucoPoseArray()
        
        # Header 설정
        poses_array.header = Header()
        poses_array.header.stamp = self.get_clock().now().to_msg()
        poses_array.header.frame_id = self.map_frame
        
        # 각 마커에 대한 ArucoPose 생성
        aruco_poses = []
        for marker in detected_markers:
            marker_id = marker["id"]
            if marker_id in self.robot_states and self.robot_states[marker_id].initialized:
                state = self.robot_states[marker_id]
                
                aruco_pose = ArucoPose()
                aruco_pose.id = marker_id
                aruco_pose.x = float(state.x)
                aruco_pose.y = float(state.y)
                aruco_pose.yaw = float(state.yaw)
                
                aruco_poses.append(aruco_pose)
        
        poses_array.poses = aruco_poses
        poses_array.count = len(aruco_poses)
        
        return poses_array

    def process_frame(self):
        start_time = time.time()
        
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Camera frame not received!")
            return

        detect_start = time.time()
        current_time = time.time()
        detected = self.detector.detect_markers(frame)
        detect_end = time.time()
        detect_time_ms = (detect_end - detect_start) * 1000

        processing_start = time.time()
        
        for marker in detected:
            marker_id = marker["id"]
            x, y = marker["robot_xy"]
            yaw = marker["yaw"]

            # 원본 데이터를 필터링하여 상태 업데이트
            self.update_robot_state(marker_id, x, y, yaw, current_time)

        # 모든 감지된 마커들을 배열로 발행
        if detected:
            aruco_poses_array_msg = self.create_aruco_poses_array_message(detected)
            self.aruco_poses_array_publisher.publish(aruco_poses_array_msg)

        processing_end = time.time()
        processing_time_ms = (processing_end - processing_start) * 1000

        display_start = time.time()
        cv2.imshow("Aruco Odometry Publisher", frame)
        cv2.waitKey(1)
        display_end = time.time()
        display_time_ms = (display_end - display_start) * 1000

        end_time = time.time()
        total_time_ms = (end_time - start_time) * 1000

        # 성능 정보 출력 (원본 코드와 동일)
        if len(detected) > 0:  # 마커가 감지된 경우만 출력
            print(f"=== process_frame 성능 측정 ===")
            print(f"마커 감지: {detect_time_ms:.2f} ms")
            print(f"마커 처리: {processing_time_ms:.2f} ms")
            print(f"화면 출력: {display_time_ms:.2f} ms")
            print(f"전체 시간: {total_time_ms:.2f} ms ({len(detected)}개 마커)")
            print(f"FPS: {1000/total_time_ms:.1f}")
            print("=" * 30)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    import os
    from pathlib import Path
    rclpy.init()

    # 현재 파일 기준으로 include 경로 계산
    script_path = Path(__file__).resolve()
    package_root = script_path.parent.parent  # aruco_marker_pkg/
    base_path = package_root / 'include'

    # 파일 경로 설정
    camera_matrix_path = base_path / 'camera_matrix.npy'
    dist_coeffs_path = base_path / 'dist_coeffs.npy'

    # 존재 여부 확인
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
            rclpy.spin_once(node, timeout_sec=0.005)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()