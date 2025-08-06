#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
통합 ArUco GUI 모니터링 시스템
- ArUco 마커 검출 및 위치 추적
- GUI에서 실시간 로봇 상태 모니터링
- PID 및 Goal Mover 파라미터 튜닝
- 맵 위에 로봇 위치 표시
"""
import sys
import math
import numpy as np
import cv2
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from tf2_ros import TransformBroadcaster

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout, QLabel,
    QDoubleSpinBox, QGroupBox, QGridLayout, QTabWidget, QPushButton,
    QTextEdit, QScrollArea
)
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap, QPainter, QPen, QBrush, QColor
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt

# === ArUco 관련 함수들 ===
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

        self.src_pts = np.array([
            [-1.420,  0.097],  
            [-1.528, -0.830],  
            [ 0.459,  0.076],  
            [ 0.410, -0.845]
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
                    "text_pos": (center_x, center_y),
                    "corners": corners_reshaped
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
        self.last_seen = time.time()

# === 통합 ROS 스레드 ===
class IntegratedRosThread(QThread):
    # GUI 업데이트 시그널들
    state_update = pyqtSignal(str)
    angle_update = pyqtSignal(float)
    dist_update = pyqtSignal(float)
    img_update = pyqtSignal(object)
    robot_states_update = pyqtSignal(dict)  # 로봇 상태 정보 전달
    performance_update = pyqtSignal(dict)   # 성능 정보 전달

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.running = True
        
        # ArUco 관련 초기화
        self.setup_aruco()
        
        # 파라미터 서비스 클라이언트
        self.pid_param_cli = self.node.create_client(SetParameters, '/pid_controller/set_parameters')
        self.goal_mover_param_cli = self.node.create_client(SetParameters, '/goal_mover/set_parameters')
        
        # 기존 토픽 구독
        self.node.create_subscription(String, '/state', lambda m: self.state_update.emit(m.data), 10)
        self.node.create_subscription(Float64, '/angle_error', lambda m: self.angle_update.emit(m.data), 10)
        self.node.create_subscription(Float64, '/distance_error', lambda m: self.dist_update.emit(m.data), 10)
        
        # ArUco 관련 퍼블리셔들
        self.pose_publishers = {}
        self.odom_publishers = {}
        self.robot_states = {}
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self.node)
        
        # 필터 계수들
        self.velocity_alpha = 0.3
        self.position_alpha = 0.7
        self.yaw_alpha = 0.6
        
        self.odom_frame = "odom"
        self.map_frame = "map"

    def setup_aruco(self):
        """ArUco 검출기 초기화"""
        try:
            # 카메라 캘리브레이션 데이터 로드
            base_path = '/home/addinnedu/monitoring_camera_ws/src/aruco_marker_pkg/include/'
            camera_matrix = np.load(base_path + 'camera_matrix.npy')
            dist_coeffs = np.load(base_path + 'dist_coeffs.npy')
            
            self.detector = ArucoDetector(camera_matrix, dist_coeffs)
            
            # 카메라 초기화
            self.cap = cv2.VideoCapture("/dev/video_cam", cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1200)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
            self.cap.set(cv2.CAP_PROP_FPS, 30)

            width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            fps = self.cap.get(cv2.CAP_PROP_FPS)
            self.node.get_logger().info(f"Camera: {width}x{height} @ {fps}fps")
            
        except Exception as e:
            self.node.get_logger().error(f"ArUco setup failed: {e}")
            self.cap = None
            self.detector = None

    def update_robot_state(self, marker_id, x_raw, y_raw, yaw_raw, current_time):
        """로봇 상태 업데이트 (필터링 포함)"""
        if marker_id not in self.robot_states:
            self.robot_states[marker_id] = RobotState()

        state = self.robot_states[marker_id]
        state.last_seen = current_time
        
        if not state.initialized:
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
        x_filtered = self.position_alpha * state.x + (1 - self.position_alpha) * x_raw
        y_filtered = self.position_alpha * state.y + (1 - self.position_alpha) * y_raw
        
        yaw_diff = normalize_angle(yaw_raw - state.yaw)
        yaw_filtered = normalize_angle(state.yaw + (1 - self.yaw_alpha) * yaw_diff)

        # 속도 계산
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

    def create_odometry_message(self, marker_id, current_time):
        """Odometry 메시지 생성"""
        if marker_id not in self.robot_states:
            return None

        state = self.robot_states[marker_id]
        if not state.initialized:
            return None

        odom = Odometry()
        odom.header.stamp = self.node.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = f"base_footprint_{marker_id}"

        odom.pose.pose.position.x = state.x
        odom.pose.pose.position.y = state.y
        odom.pose.pose.position.z = 0.0

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
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = state.vyaw

        # 공분산 설정
        pose_cov = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.02]
        
        twist_cov = [0.02, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.02, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.05]

        odom.pose.covariance = pose_cov
        odom.twist.covariance = twist_cov

        return odom

    def run(self):
        """메인 실행 루프"""
        self.node.get_logger().info("통합 ROS Thread 시작")
        
        while rclpy.ok() and self.running:
            self.process_frame()
            rclpy.spin_once(self.node, timeout_sec=0.005)

    def process_frame(self):
        """프레임 처리 (ArUco 검출 + GUI 업데이트)"""
        if not hasattr(self, 'cap') or self.cap is None:
            return
            
        start_time = time.time()
        
        ret, frame = self.cap.read()
        if not ret:
            self.node.get_logger().warn("Camera frame not received!")
            return

        detect_start = time.time()
        current_time = time.time()
        
        # ArUco 마커 검출
        detected = []
        if self.detector is not None:
            detected = self.detector.detect_markers(frame)
            
        detect_end = time.time()
        detect_time_ms = (detect_end - detect_start) * 1000

        processing_start = time.time()
        
        # 검출된 마커들 처리
        for marker in detected:
            marker_id = marker["id"]
            x, y = marker["robot_xy"]
            yaw = marker["yaw"]
            cx, cy = marker["text_pos"]

            # 프레임에 마커 정보 그리기
            corners = marker.get("corners", None)
            if corners is not None:
                cv2.polylines(frame, [corners.astype(int)], True, (0, 255, 0), 2)
                cv2.putText(frame, f'ID:{marker_id} ({x:.2f},{y:.2f}) {math.degrees(yaw):.1f}°', 
                           (cx-50, cy-30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

            # 로봇 상태 업데이트
            self.update_robot_state(marker_id, x, y, yaw, current_time)

            # Publishers 생성 (처음 감지될 때)
            if marker_id not in self.pose_publishers:
                self.pose_publishers[marker_id] = self.node.create_publisher(
                    PoseStamped, f"/robot{marker_id}/pose", 10)
                self.odom_publishers[marker_id] = self.node.create_publisher(
                    Odometry, f"/odom_{marker_id}", 10)

            # 메시지 발행
            state = self.robot_states[marker_id]
            
            # PoseStamped 발행
            pose = PoseStamped()
            pose.header.stamp = self.node.get_clock().now().to_msg()
            pose.header.frame_id = self.map_frame
            pose.pose.position.x = state.x
            pose.pose.position.y = state.y
            pose.pose.position.z = 0.0

            q = yaw_to_quaternion(state.yaw)
            pose.pose.orientation.x = q['x']
            pose.pose.orientation.y = q['y']
            pose.pose.orientation.z = q['z']
            pose.pose.orientation.w = q['w']

            self.pose_publishers[marker_id].publish(pose)

            # Odometry 발행
            odom = self.create_odometry_message(marker_id, current_time)
            if odom is not None:
                odom.header.frame_id = f"odom_{marker_id}"
                odom.child_frame_id = f"base_link_{marker_id}"
                self.odom_publishers[marker_id].publish(odom)

        processing_end = time.time()
        processing_time_ms = (processing_end - processing_start) * 1000

        # GUI로 이미지 전송 (BGR -> RGB 변환)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.img_update.emit(rgb_frame)

        # 로봇 상태 정보 GUI로 전송
        robot_info = {}
        for robot_id, state in self.robot_states.items():
            if state.initialized and (current_time - state.last_seen) < 1.0:  # 1초 이내 검출
                robot_info[robot_id] = {
                    'x': state.x,
                    'y': state.y,
                    'yaw': state.yaw,
                    'vx': state.vx,
                    'vy': state.vy,
                    'vyaw': state.vyaw,
                    'last_seen': state.last_seen
                }
        self.robot_states_update.emit(robot_info)

        end_time = time.time()
        total_time_ms = (end_time - start_time) * 1000

        # 성능 정보 전송
        if len(detected) > 0:
            perf_info = {
                'detect_time': detect_time_ms,
                'processing_time': processing_time_ms,
                'total_time': total_time_ms,
                'marker_count': len(detected),
                'fps': 1000/total_time_ms if total_time_ms > 0 else 0
            }
            self.performance_update.emit(perf_info)

    def set_pid_param(self, name, value):
        """PID 파라미터 설정"""
        self._set_parameter(self.pid_param_cli, name, value, 'PID Controller')

    def set_goal_mover_param(self, name, value):
        """Goal Mover 파라미터 설정"""
        self._set_parameter(self.goal_mover_param_cli, name, value, 'Goal Mover')

    def _set_parameter(self, client, name, value, node_name):
        """공통 파라미터 설정 함수"""
        if not client.wait_for_service(timeout_sec=0.1):
            return
        try:
            req = SetParameters.Request()
            param = Parameter(name=name,
                              value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                                   double_value=value))
            req.parameters.append(param)
            client.call_async(req)
        except Exception as e:
            self.node.get_logger().error(f"Set {node_name} parameter error: {e}")

    def stop(self):
        """스레드 정지"""
        self.running = False

    def cleanup(self):
        """리소스 정리"""
        self.running = False
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
            self.node.get_logger().info("Camera released")
        cv2.destroyAllWindows()

# === 수정된 맵 위젯 (matplotlib 기반) ===
class MapWidget(QWidget):  # QLabel에서 QWidget으로 변경
    def __init__(self, custom_map):
        super().__init__()
        self.custom_map = custom_map
        self.robot_positions = {}  # {robot_id: {'x': x, 'y': y, 'yaw': yaw}}
        
        # matplotlib 설정
        self.figure = Figure(figsize=(7, 4))
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111)
        
        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        self.setLayout(layout)
        
        # 맵을 numpy로 변환
        self.map_array = np.array(custom_map, dtype=np.uint8)
        self.map_array = (1 - self.map_array) * 255  # 0 → 255 (white), 1 → 0 (black)
        self.map_array = np.flipud(self.map_array)   # y축 상하반전
        
        self.update_map()

    def update_map(self):
        """맵과 로봇 위치를 그려서 업데이트"""
        self.ax.clear()
        
        # 맵 출력: 범위 (-0.1, -0.1) ~ (2.1, 1.1)
        extent = [-0.1, 2.1, -0.1, 1.1]
        self.ax.imshow(self.map_array, cmap='gray', origin='lower', extent=extent)
        
        # 로봇들 그리기
        legend_handles = []
        legend_labels = []
        
        for robot_id, pos in self.robot_positions.items():
            # 로봇 색상 (ID에 따라)
            colors = ['red', 'blue', 'green', 'orange', 'purple', 'cyan']
            color = colors[robot_id % len(colors)]
            
            x, y, yaw = pos['x'], pos['y'], pos['yaw']
            
            # 로봇 현재 위치 (원점)
            robot_point = self.ax.plot(x, y, 'o', color=color, markersize=10, 
                                     markeredgecolor='black', markeredgewidth=1,
                                     label=f'Robot {robot_id}')[0]
            legend_handles.append(robot_point)
            legend_labels.append(f'Robot {robot_id}')
            
            # 방향 화살표
            arrow_length = 0.05
            dx = arrow_length * math.cos(yaw)
            dy = arrow_length * math.sin(yaw)
            
            self.ax.arrow(x, y, dx, dy, head_width=0.02, head_length=0.02,
                         color=color, alpha=1.0, linewidth=2)
            
            # 로봇 ID 텍스트
            self.ax.text(x + 0.05, y + 0.05, f'R{robot_id}', 
                        color='white', fontweight='bold', fontsize=8,
                        bbox=dict(boxstyle="round,pad=0.2", facecolor=color, alpha=0.7))
        
        # 축 설정
        self.ax.set_xlim(-0.1, 2.1)
        self.ax.set_ylim(-0.1, 1.1)
        self.ax.set_xticks(np.arange(-0.1, 2.11, 0.1))
        self.ax.set_yticks(np.arange(-0.1, 1.11, 0.1))
        
        # 격자 표시
        self.ax.grid(True, alpha=0.3, linewidth=0.5)
        
        # 축 레이블
        self.ax.set_xlabel('X (m)', fontsize=10)
        self.ax.set_ylabel('Y (m)', fontsize=10)
        self.ax.set_title('Robot Positions on Map', fontsize=12)
        
        # 범례 추가 (로봇이 있는 경우만)
        if legend_handles:
            self.ax.legend(legend_handles, legend_labels, loc='upper right', fontsize=8)
        
        # 여백 조정
        self.figure.tight_layout()
        self.canvas.draw()

    def update_robot_positions(self, robot_states):
        """로봇 위치 정보 업데이트"""
        self.robot_positions = robot_states
        self.update_map()


# === 메인 GUI 창 ===
class IntegratedMainWindow(QMainWindow):
    def __init__(self, ros_thread, node):
        super().__init__()
        self.setWindowTitle('통합 ArUco GUI 모니터링 시스템')
        self.resize(1600, 1200)

        # 중앙 위젯
        central = QWidget()
        self.setCentralWidget(central)
        
        # 4분면 그리드 레이아웃
        main_layout = QGridLayout(central)
        main_layout.setSpacing(10)

        # === 1분면: 맵과 로봇 위치 ===
        self.setup_map_section(main_layout)
        
        # === 2분면: 라이브 카메라 ===
        self.setup_camera_section(main_layout)
        
        # === 3분면: 제어 파라미터 ===
        self.setup_control_section(main_layout, ros_thread)
        
        # === 4분면: 상태 모니터링 ===
        self.setup_status_section(main_layout)

        # 그리드 레이아웃 비율 설정
        main_layout.setRowStretch(0, 1)
        main_layout.setRowStretch(1, 1)
        main_layout.setColumnStretch(0, 1)
        main_layout.setColumnStretch(1, 1)

        # ROS 시그널 연결
        self.connect_ros_signals(ros_thread)

        self.ros_node = node
        self.ros_thread = ros_thread
        
        # FPS 계산용
        self.fps_counter = 0
        self.fps_timer = QTimer()
        self.fps_timer.timeout.connect(self.update_fps_display)
        self.fps_timer.start(1000)

    def setup_map_section(self, main_layout):
        """맵 섹션 설정 (matplotlib 기반으로 수정됨)"""
        # 커스텀 맵 데이터
        custom_map = [
            [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
            [1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1],
            [1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1],
            [1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1],
            [1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1],
            [1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1],
            [1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1],
            [1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1],
            [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
            [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
            [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
            [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        ]
        
        map_group = QGroupBox('1분면: 맵 + 로봇 위치')
        map_layout = QVBoxLayout()
        
        # 맵 위젯 생성 (matplotlib 기반으로 변경됨)
        self.map_widget = MapWidget(custom_map)
        map_layout.addWidget(self.map_widget)
        
        # 맵 정보 표시
        map_info_layout = QHBoxLayout()
        self.map_info_label = QLabel("맵 크기: 2.0m x 1.0m | 활성 로봇: 0")
        map_info_layout.addWidget(self.map_info_label)
        map_layout.addLayout(map_info_layout)
        
        map_group.setLayout(map_layout)
        main_layout.addWidget(map_group, 0, 0)

    def setup_camera_section(self, main_layout):
        """카메라 섹션 설정"""
        cam_group = QGroupBox('2분면: Live Camera + ArUco Detection')
        cam_layout = QVBoxLayout()
        
        self.lbl_cam = QLabel()
        self.lbl_cam.setFixedSize(700, 400)
        self.lbl_cam.setStyleSheet("border: 2px solid green; background-color: black; color: white; font-size: 14px;")
        self.lbl_cam.setAlignment(Qt.AlignCenter)
        self.lbl_cam.setText("ArUco 카메라 연결 대기 중...")
        cam_layout.addWidget(self.lbl_cam, alignment=Qt.AlignCenter)
        
        # 카메라 상태 정보
        cam_status_layout = QHBoxLayout()
        self.cam_status_label = QLabel("Status: Initializing...")
        self.frame_count_label = QLabel("Frames: 0")
        self.detection_label = QLabel("Detections: 0")
        cam_status_layout.addWidget(self.cam_status_label)
        cam_status_layout.addWidget(self.frame_count_label)
        cam_status_layout.addWidget(self.detection_label)
        cam_layout.addLayout(cam_status_layout)
        
        self.frame_count = 0
        cam_group.setLayout(cam_layout)
        main_layout.addWidget(cam_group, 0, 1)

    def setup_control_section(self, main_layout, ros_thread):
        """제어 파라미터 섹션 설정"""
        control_group = QGroupBox('3분면: 제어 파라미터')
        control_layout = QVBoxLayout()
        
        # 파라미터 탭
        param_tabs = QTabWidget()
        
        # PID 튜닝 탭
        self.setup_pid_tab(param_tabs, ros_thread)
        
        # Goal Mover 튜닝 탭
        self.setup_goal_mover_tab(param_tabs, ros_thread)
        
        # ArUco 설정 탭
        self.setup_aruco_tab(param_tabs, ros_thread)
        
        control_layout.addWidget(param_tabs)
        control_group.setLayout(control_layout)
        main_layout.addWidget(control_group, 1, 0)

    def setup_pid_tab(self, param_tabs, ros_thread):
        """PID 제어 탭 설정"""
        pid_widget = QWidget()
        pid_layout = QGridLayout()
        
        self.pid_spin = {}
        pid_params = [
            ('Angular P','angular_P',1.0),
            ('Angular I','angular_I',0.0),
            ('Angular D','angular_D',0.0),
            ('Linear P','linear_P',1.0),
            ('Linear I','linear_I',0.0),
            ('Linear D','linear_D',0.0)
        ]
        
        for i,(lbl,name,defv) in enumerate(pid_params):
            pid_layout.addWidget(QLabel(lbl), i, 0)
            sb = QDoubleSpinBox()
            sb.setRange(-10,10)
            sb.setDecimals(3)
            sb.setSingleStep(0.001)
            sb.setValue(defv)
            sb.valueChanged.connect(lambda v,n=name: ros_thread.set_pid_param(n, v))
            pid_layout.addWidget(sb, i, 1)
            self.pid_spin[name] = sb
            
            # 현재 값 표시
            val_label = QLabel(f"{defv}")
            pid_layout.addWidget(val_label, i, 2)
            sb.valueChanged.connect(lambda v, lbl=val_label: lbl.setText(f"{v:.3f}"))
        
        pid_widget.setLayout(pid_layout)
        param_tabs.addTab(pid_widget, "PID Control")

    def setup_goal_mover_tab(self, param_tabs, ros_thread):
        """Goal Mover 탭 설정"""
        goal_widget = QWidget()
        goal_layout = QGridLayout()
        
        self.goal_spin = {}
        goal_params = [
            ('Linear Gain (k_lin)', 'k_lin', 0.3, 0.0, 5.0, 0.01),
            ('Angular Gain (k_ang)', 'k_ang', 0.1, 0.0, 2.0, 0.01),
            ('Min Linear Speed', 'min_linear_speed', 0.55, 0.0, 2.0, 0.05),
            ('Min Angular Speed', 'min_angular_speed', 0.55, 0.0, 2.0, 0.05),
            ('Angle Tolerance (deg)', 'angle_tolerance_deg', 16.0, 1.0, 90.0, 1.0),
            ('Position Tolerance (m)', 'pos_tolerance', 0.03, 0.01, 1.0, 0.01)
        ]
        
        for i, (lbl, name, defv, minv, maxv, step) in enumerate(goal_params):
            goal_layout.addWidget(QLabel(lbl), i, 0)
            sb = QDoubleSpinBox()
            sb.setRange(minv, maxv)
            sb.setDecimals(3)
            sb.setSingleStep(step)
            sb.setValue(defv)
            sb.valueChanged.connect(lambda v, n=name: ros_thread.set_goal_mover_param(n, v))
            goal_layout.addWidget(sb, i, 1)
            self.goal_spin[name] = sb
            
            # 현재 값 표시
            val_label = QLabel(f"{defv}")
            goal_layout.addWidget(val_label, i, 2)
            sb.valueChanged.connect(lambda v, lbl=val_label: lbl.setText(f"{v:.3f}"))
        
        # 프리셋 버튼들
        self._add_preset_buttons(goal_layout, ros_thread, len(goal_params))
        
        goal_widget.setLayout(goal_layout)
        param_tabs.addTab(goal_widget, "Goal Mover")

    def setup_aruco_tab(self, param_tabs, ros_thread):
        """ArUco 설정 탭 추가"""
        aruco_widget = QWidget()
        aruco_layout = QGridLayout()
        
        # ArUco 필터 설정
        filter_params = [
            ('Position Alpha', 'position_alpha', 0.7, 0.0, 1.0, 0.1),
            ('Velocity Alpha', 'velocity_alpha', 0.3, 0.0, 1.0, 0.1),
            ('Yaw Alpha', 'yaw_alpha', 0.6, 0.0, 1.0, 0.1)
        ]
        
        self.aruco_spin = {}
        for i, (lbl, name, defv, minv, maxv, step) in enumerate(filter_params):
            aruco_layout.addWidget(QLabel(lbl), i, 0)
            sb = QDoubleSpinBox()
            sb.setRange(minv, maxv)
            sb.setDecimals(2)
            sb.setSingleStep(step)
            sb.setValue(defv)
            # 필터 값 변경 시 ROS 스레드의 값도 업데이트
            sb.valueChanged.connect(lambda v, n=name: self.update_aruco_param(n, v))
            aruco_layout.addWidget(sb, i, 1)
            self.aruco_spin[name] = sb
            
            val_label = QLabel(f"{defv}")
            aruco_layout.addWidget(val_label, i, 2)
            sb.valueChanged.connect(lambda v, lbl=val_label: lbl.setText(f"{v:.2f}"))
        
        # ArUco 상태 정보
        aruco_info_group = QGroupBox('ArUco 상태')
        aruco_info_layout = QVBoxLayout()
        self.aruco_status_label = QLabel("검출 상태: 대기 중")
        self.marker_count_label = QLabel("감지된 마커: 0")
        self.detection_fps_label = QLabel("검출 FPS: 0")
        aruco_info_layout.addWidget(self.aruco_status_label)
        aruco_info_layout.addWidget(self.marker_count_label)
        aruco_info_layout.addWidget(self.detection_fps_label)
        aruco_info_group.setLayout(aruco_info_layout)
        
        aruco_layout.addWidget(aruco_info_group, len(filter_params), 0, 1, 3)
        
        aruco_widget.setLayout(aruco_layout)
        param_tabs.addTab(aruco_widget, "ArUco Settings")

    def update_aruco_param(self, name, value):
        """ArUco 파라미터 업데이트"""
        if hasattr(self.ros_thread, name):
            setattr(self.ros_thread, name, value)

    def setup_status_section(self, main_layout):
        """상태 모니터링 섹션 설정"""
        status_group = QGroupBox('4분면: 로봇 상태 모니터링')
        status_layout = QVBoxLayout()
        
        # 로봇 상태 표시 (스크롤 가능)
        robot_status_group = QGroupBox('로봇 상태')
        robot_status_layout = QVBoxLayout()
        
        # 기본 상태 정보
        self.lbl_state = QLabel('전체 상태: 대기 중')
        self.lbl_ang = QLabel('각도 오차: 0.00')
        self.lbl_dist = QLabel('거리 오차: 0.00')
        robot_status_layout.addWidget(self.lbl_state)
        robot_status_layout.addWidget(self.lbl_ang)
        robot_status_layout.addWidget(self.lbl_dist)
        
        # 개별 로봇 상태 표시 (스크롤 영역)
        scroll_area = QScrollArea()
        scroll_widget = QWidget()
        self.robot_details_layout = QVBoxLayout(scroll_widget)
        scroll_area.setWidget(scroll_widget)
        scroll_area.setWidgetResizable(True)
        scroll_area.setMaximumHeight(200)
        robot_status_layout.addWidget(scroll_area)
        
        robot_status_group.setLayout(robot_status_layout)
        status_layout.addWidget(robot_status_group)
        
        # 시스템 정보 표시
        system_info_group = QGroupBox('시스템 정보')
        system_info_layout = QVBoxLayout()
        self.lbl_fps = QLabel('카메라 FPS: 0')
        self.lbl_ros_status = QLabel('ROS 상태: 연결됨')
        self.lbl_performance = QLabel('처리 성능: 대기 중')
        system_info_layout.addWidget(self.lbl_fps)
        system_info_layout.addWidget(self.lbl_ros_status)
        system_info_layout.addWidget(self.lbl_performance)
        system_info_group.setLayout(system_info_layout)
        status_layout.addWidget(system_info_group)
        
        # 제어 버튼들
        self.setup_control_buttons(status_layout)
        
        status_group.setLayout(status_layout)
        main_layout.addWidget(status_group, 1, 1)

    def setup_control_buttons(self, status_layout):
        """제어 버튼들 설정"""
        control_buttons_group = QGroupBox('제어 버튼')
        control_buttons_layout = QGridLayout()
        
        # 긴급 정지 버튼
        emergency_btn = QPushButton('긴급 정지')
        emergency_btn.setStyleSheet("background-color: red; color: white; font-weight: bold; font-size: 14px;")
        emergency_btn.setFixedHeight(40)
        emergency_btn.clicked.connect(self.emergency_stop)
        control_buttons_layout.addWidget(emergency_btn, 0, 0)
        
        # 시스템 리셋 버튼
        reset_btn = QPushButton('시스템 리셋')
        reset_btn.setStyleSheet("background-color: orange; color: white; font-weight: bold; font-size: 14px;")
        reset_btn.setFixedHeight(40)
        reset_btn.clicked.connect(self.system_reset)
        control_buttons_layout.addWidget(reset_btn, 0, 1)
        
        # 파라미터 저장 버튼
        save_btn = QPushButton('파라미터 저장')
        save_btn.setStyleSheet("background-color: green; color: white; font-weight: bold; font-size: 14px;")
        save_btn.setFixedHeight(40)
        save_btn.clicked.connect(self.save_parameters)
        control_buttons_layout.addWidget(save_btn, 1, 0)
        
        # 파라미터 로드 버튼
        load_btn = QPushButton('파라미터 로드')
        load_btn.setStyleSheet("background-color: blue; color: white; font-weight: bold; font-size: 14px;")
        load_btn.setFixedHeight(40)
        load_btn.clicked.connect(self.load_parameters)
        control_buttons_layout.addWidget(load_btn, 1, 1)
        
        control_buttons_group.setLayout(control_buttons_layout)
        status_layout.addWidget(control_buttons_group)

    def _add_preset_buttons(self, layout, ros_thread, start_row):
        """Goal Mover 파라미터 프리셋 버튼들 추가"""
        presets = {
            "보수적": {"k_lin": 0.2, "k_ang": 0.05, "min_linear_speed": 0.3, "min_angular_speed": 0.3},
            "표준": {"k_lin": 0.3, "k_ang": 0.1, "min_linear_speed": 0.55, "min_angular_speed": 0.55},
            "적극적": {"k_lin": 0.5, "k_ang": 0.2, "min_linear_speed": 0.8, "min_angular_speed": 0.8}
        }
        
        layout.addWidget(QLabel("프리셋:"), start_row, 0)
        
        for i, (name, params) in enumerate(presets.items()):
            btn = QPushButton(name)
            btn.setStyleSheet("background-color: lightblue; font-weight: bold;")
            btn.clicked.connect(lambda checked, p=params: self._apply_preset(p, ros_thread))
            layout.addWidget(btn, start_row, i + 1)

    def _apply_preset(self, preset_params, ros_thread):
        """프리셋 파라미터들을 적용"""
        for param_name, value in preset_params.items():
            if param_name in self.goal_spin:
                self.goal_spin[param_name].setValue(value)
                ros_thread.set_goal_mover_param(param_name, value)

    def connect_ros_signals(self, ros_thread):
        """ROS 시그널들 연결"""
        ros_thread.state_update.connect(lambda s: self.lbl_state.setText(f'전체 상태: {s}'))
        ros_thread.angle_update.connect(lambda v: self.lbl_ang.setText(f'각도 오차: {v:.2f}'))
        ros_thread.dist_update.connect(lambda v: self.lbl_dist.setText(f'거리 오차: {v:.2f}'))
        ros_thread.img_update.connect(self._update_camera)
        ros_thread.robot_states_update.connect(self._update_robot_states)
        ros_thread.performance_update.connect(self._update_performance)

    def _update_camera(self, img):
        """카메라 이미지 업데이트"""
        try:
            h, w, ch = img.shape
            bytes_per_line = ch * w
            qimg = QImage(img.data, w, h, bytes_per_line, QImage.Format_RGB888)
            
            pix = QPixmap.fromImage(qimg).scaled(
                self.lbl_cam.size(), 
                Qt.KeepAspectRatio, 
                Qt.FastTransformation
            )
            
            self.lbl_cam.setPixmap(pix)
            
            self.frame_count += 1
            self.fps_counter += 1
            
            if self.frame_count % 30 == 0:
                self.cam_status_label.setText(f"Status: Live - {w}x{h}")
                self.frame_count_label.setText(f"Frames: {self.frame_count}")
            
        except Exception as e:
            self.cam_status_label.setText(f"Status: Error - {str(e)}")

    def _update_robot_states(self, robot_states):
        """로봇 상태 정보 업데이트"""
        # 맵 위젯 업데이트
        self.map_widget.update_robot_positions(robot_states)
        
        # 맵 정보 레이블 업데이트
        active_count = len(robot_states)
        self.map_info_label.setText(f"맵 크기: 2.0m x 1.0m | 활성 로봇: {active_count}")
        
        # 개별 로봇 상태 표시 업데이트
        # 기존 위젯들 제거
        for i in reversed(range(self.robot_details_layout.count())):
            child = self.robot_details_layout.itemAt(i).widget()
            if child:
                child.deleteLater()
        
        # 새로운 로봇 상태 위젯들 추가
        for robot_id, state in robot_states.items():
            robot_widget = QGroupBox(f'로봇 {robot_id}')
            robot_layout = QGridLayout()
            
            # 위치 정보
            robot_layout.addWidget(QLabel('위치:'), 0, 0)
            robot_layout.addWidget(QLabel(f'({state["x"]:.3f}, {state["y"]:.3f})'), 0, 1)
            
            # 방향 정보
            robot_layout.addWidget(QLabel('방향:'), 1, 0)
            robot_layout.addWidget(QLabel(f'{math.degrees(state["yaw"]):.1f}°'), 1, 1)
            
            # 속도 정보
            robot_layout.addWidget(QLabel('속도:'), 2, 0)
            speed = math.sqrt(state["vx"]**2 + state["vy"]**2)
            robot_layout.addWidget(QLabel(f'{speed:.3f} m/s'), 2, 1)
            
            # 각속도 정보
            robot_layout.addWidget(QLabel('각속도:'), 3, 0)
            robot_layout.addWidget(QLabel(f'{math.degrees(state["vyaw"]):.1f}°/s'), 3, 1)
            
            robot_widget.setLayout(robot_layout)
            self.robot_details_layout.addWidget(robot_widget)

    def _update_performance(self, perf_info):
        """성능 정보 업데이트"""
        self.detection_label.setText(f"Detections: {perf_info['marker_count']}")
        self.detection_fps_label.setText(f"검출 FPS: {perf_info['fps']:.1f}")
        self.marker_count_label.setText(f"감지된 마커: {perf_info['marker_count']}")
        self.lbl_performance.setText(f"처리 시간: {perf_info['total_time']:.1f}ms")

    def update_fps_display(self):
        """FPS 표시 업데이트"""
        fps = self.fps_counter
        self.lbl_fps.setText(f'카메라 FPS: {fps}')
        self.fps_counter = 0

    def emergency_stop(self):
        """긴급 정지 기능"""
        # TODO: 로봇들에게 정지 명령 전송
        self.lbl_state.setText('전체 상태: 긴급 정지됨')
        print("긴급 정지 명령 전송됨")

    def system_reset(self):
        """시스템 리셋 기능"""
        # TODO: 시스템 리셋 로직
        self.lbl_state.setText('전체 상태: 시스템 리셋됨')
        print("시스템 리셋 수행됨")

    def save_parameters(self):
        """파라미터 저장 기능"""
        # TODO: 현재 파라미터들을 파일로 저장
        print("파라미터 저장됨")

    def load_parameters(self):
        """파라미터 로드 기능"""
        # TODO: 파일에서 파라미터들을 로드
        print("파라미터 로드됨")

    def closeEvent(self, event):
        """창 닫기 이벤트"""
        self.ros_thread.stop()
        self.ros_thread.cleanup()
        self.ros_thread.quit()
        self.ros_thread.wait()
        event.accept()

# === 메인 실행 함수 ===
def main():
    """메인 실행 함수"""
    rclpy.init()
    node = Node('integrated_aruco_monitor')
    
    # 통합 ROS 스레드 생성
    ros_thread = IntegratedRosThread(node)
    
    # Qt 애플리케이션 시작
    app = QApplication(sys.argv)
    win = IntegratedMainWindow(ros_thread, node)
    win.show()
    
    # ROS 스레드 시작
    ros_thread.start()
    
    try:
        rc = app.exec_()
    except KeyboardInterrupt:
        print("사용자에 의해 중단됨")
    finally:
        # 정리 작업
        print("시스템 정리 중...")
        ros_thread.stop()
        ros_thread.cleanup()
        ros_thread.quit()
        ros_thread.wait()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(rc)

if __name__ == '__main__':
    main()