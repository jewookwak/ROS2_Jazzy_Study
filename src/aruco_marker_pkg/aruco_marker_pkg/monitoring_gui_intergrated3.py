#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
통합 AMR 제어 및 모니터링 시스템 - 단일 파일 버전
ArUco 마커 검출, GUI 제어, ROS 통신을 모두 포함
"""

import math
import numpy as np
import cv2
import time
import os
import yaml
import sys
import threading
from pathlib import Path
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist 
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from tf2_ros import TransformBroadcaster

from PyQt5.QtCore import QThread, pyqtSignal, Qt, QTimer, QRect, QPoint
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QDoubleSpinBox, QGroupBox, QGridLayout, QTabWidget, QPushButton,
    QTextEdit, QScrollArea, QCheckBox, QButtonGroup, QRadioButton
)
from PyQt5.QtGui import (
    QImage, QPixmap, QPainter, QPen, QBrush, QColor, QFont, QPolygon,
    QFontDatabase
)

from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.patches import FancyBboxPatch

# Flask for camera streaming
from flask import Flask, Response

from robocallee_fms.msg import ArucoPose, ArucoPoseArray
from std_msgs.msg import Header
# ==============================================================================
# 1. ArUco Detection and Utility Functions
# ==============================================================================

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
            [-0.699,  0.081],  
            [-0.766, -0.371],  
            [ 0.222,  0.071],  
            [ 0.187, -0.369]
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
        self.current_state = "IDLE"
        self.angle_error = 0.0
        self.distance_error = 0.0
        self.trajectory = []

# ==============================================================================
# 2. Map Widget
# ==============================================================================

class MapWidget(QLabel):
    """맵 위젯 - 로봇 위치와 경로를 표시"""
    def __init__(self, custom_map):
        super().__init__()
        self.custom_map = custom_map
        self.robot_positions = {}
        self.selected_robot_id = None
        self.target_pose = None
        self.drag_start = None
        self.drag_current = None
        self.trajectory_data = {}
        
        self.setFixedSize(800, 400)
        self.setStyleSheet("border: 2px solid blue;")

        # 월드 스케일(고정)
        self.map_width_m = 2.0
        self.map_height_m = 1.0

        # 격자/축 설정
        self.major_step_m = 0.2
        self.plot_margin_px = 34
        self.axis_font_pt = 10
        self.tick_font_pt = 9

        self.rows = len(self.custom_map)
        self.cols = len(self.custom_map[0])
        self.cell_w_m = self.map_width_m / self.cols
        self.cell_h_m = self.map_height_m / self.rows

        self.major_every_cells_x = 2
        self.major_every_cells_y = 2
        self.tick_step_m = 0.1

        # Path(트레일)
        self.show_trails = True
        self.trails = {}
        self.starts = {}
        self.max_trail_len = 300
        self.trail_min_step_m = 0.02

        # 레이아웃 튜닝
        self.legend_scale = 0.7
        self.legend_dx_cm = 3.0
        self.legend_dy_cm = 0.0
        self.map_dx_cm = -0.0
        self.map_dy_cm = 0.0
           
        self.update_map()

    def _data_rect(self, pixmap: QPixmap):
        W = pixmap.width()
        H = pixmap.height()
        l = r = t = b = self.plot_margin_px
        return QRect(l, t, W - l - r, H - t - b)

    def _cm_to_px(self, cm, axis='x'):
        dpi = self.logicalDpiX() if axis == 'x' else self.logicalDpiY()
        return int(round(cm * dpi / 2.54))

    def _wx_to_px(self, x, rect: QRect):
        return int(rect.x() + (x / self.map_width_m) * rect.width())

    def _wy_to_py(self, y, rect: QRect):
        return int(rect.y() + (1.0 - (y / self.map_height_m)) * rect.height())

    def world_to_pixel(self, world_x, world_y):
        """월드 좌표를 픽셀 좌표로 변환"""
        rect = self._data_rect(QPixmap(self.width(), self.height()))
        pixel_x = self._wx_to_px(world_x, rect)
        pixel_y = self._wy_to_py(world_y, rect)
        return pixel_x, pixel_y

    def pixel_to_world(self, pixel_x, pixel_y):
        """픽셀 좌표를 월드 좌표로 변환"""
        rect = self._data_rect(QPixmap(self.width(), self.height()))
        world_x = (pixel_x - rect.x()) / rect.width() * self.map_width_m
        world_y = (1.0 - (pixel_y - rect.y()) / rect.height()) * self.map_height_m
        return world_x, world_y

    def mousePressEvent(self, event):
        """마우스 클릭 이벤트"""
        if event.button() == Qt.LeftButton and self.selected_robot_id is not None:
            world_x, world_y = self.pixel_to_world(event.x(), event.y())
            if 0 <= world_x <= self.map_width_m and 0 <= world_y <= self.map_height_m:
                self.drag_start = (world_x, world_y)
                self.drag_current = (world_x, world_y)

    def mouseMoveEvent(self, event):
        """마우스 드래그 이벤트"""
        if self.drag_start and self.selected_robot_id is not None:
            world_x, world_y = self.pixel_to_world(event.x(), event.y())
            if 0 <= world_x <= self.map_width_m and 0 <= world_y <= self.map_height_m:
                self.drag_current = (world_x, world_y)
                self.update_map()

    def mouseReleaseEvent(self, event):
        """마우스 릴리즈 이벤트"""
        if event.button() == Qt.LeftButton and self.drag_start and self.drag_current and self.selected_robot_id is not None:
            dx = self.drag_current[0] - self.drag_start[0]
            dy = self.drag_current[1] - self.drag_start[1]
            
            if math.hypot(dx, dy) < 0.02:
                theta = 0.0
            else:
                theta = math.atan2(dy, dx)

            self.target_pose = {
                'x': self.drag_start[0],
                'y': self.drag_start[1],
                'yaw': theta,
                'robot_id': self.selected_robot_id
            }
            
            # 부모 위젯에 목표 설정 알림
            parent_widget = self.parent()
            while parent_widget:
                if hasattr(parent_widget, 'on_goal_set'):
                    parent_widget.on_goal_set(self.selected_robot_id, self.target_pose)
                    break
                parent_widget = parent_widget.parent()

        self.drag_start = None
        self.drag_current = None
        self.update_map()

    def set_selected_robot(self, robot_id):
        """선택된 로봇 설정"""
        self.selected_robot_id = robot_id
        self.update_map()

    def _draw_map_cells(self, painter: QPainter, rect: QRect):
        arr = np.array(self.custom_map, dtype=np.uint8)
        rows, cols = arr.shape
        cell_w = rect.width() / cols
        cell_h = rect.height() / rows

        painter.setPen(Qt.NoPen)

        # 빈 공간 배경
        painter.setBrush(QBrush(QColor(0, 0, 0)))
        painter.drawRect(rect)

        # 벽 칠하기
        painter.setBrush(QBrush(QColor(255, 255, 255)))
        for r in range(rows):
            y = rect.y() + int(r * cell_h)
            h = int((r+1) * cell_h) - int(r * cell_h)
            for c in range(cols):
                if arr[r, c] == 1:
                    x = rect.x() + int(c * cell_w)
                    w = int((c+1) * cell_w) - int(c * cell_w)
                    painter.drawRect(x, y, w, h)

    def _accumulate_trail(self, states: dict):
        for rid, st in states.items():
            if rid not in self.starts:
                self.starts[rid] = (st['x'], st['y'])
            path = self.trails.setdefault(rid, [])
            if not path:
                path.append((st['x'], st['y']))
            else:
                dx = st['x'] - path[-1][0]
                dy = st['y'] - path[-1][1]
                if dx*dx + dy*dy >= (self.trail_min_step_m*self.trail_min_step_m):
                    path.append((st['x'], st['y']))
                    if len(path) > self.max_trail_len:
                        del path[0:len(path)-self.max_trail_len]

    def update_map(self):
        """맵 업데이트"""
        W, H = self.width(), self.height()
        canvas = QPixmap(W, H)
        canvas.fill(QColor(20, 20, 24))
        painter = QPainter(canvas)
        painter.setRenderHint(QPainter.Antialiasing, True)

        rect = self._data_rect(canvas)
        self._draw_map_cells(painter, rect)

        # 데이터 영역 테두리
        painter.setPen(QPen(QColor(30, 30, 34), 2))
        painter.drawRect(rect)

        # 격자 그리기
        painter.setPen(QPen(QColor(200, 200, 200), 1))
        for i in range(self.cols + 1):
            px = self._wx_to_px(i * self.cell_w_m, rect)
            painter.drawLine(px, rect.top(), px, rect.bottom())
        for j in range(self.rows + 1):
            py = self._wy_to_py(j * self.cell_h_m, rect)
            painter.drawLine(rect.left(), py, rect.right(), py)

        # 두꺼운 격자
        painter.setPen(QPen(QColor(140, 175, 210), 2))
        for i in range(0, self.cols + 1, self.major_every_cells_x):
            px = self._wx_to_px(i * self.cell_w_m, rect)
            painter.drawLine(px, rect.top(), px, rect.bottom())
        for j in range(0, self.rows + 1, self.major_every_cells_y):
            py = self._wy_to_py(j * self.cell_h_m, rect)
            painter.drawLine(rect.left(), py, rect.right(), py)

        # 축/틱
        tick_font = QFont()
        tick_font.setPointSize(self.tick_font_pt)
        painter.setFont(tick_font)
        painter.setPen(QPen(QColor(220, 220, 220), 1))
        
        x = 0.0
        while x <= self.map_width_m + 1e-9:
            px = self._wx_to_px(x, rect)
            painter.drawLine(px, rect.bottom(), px, rect.bottom()+5)
            painter.drawText(px-10, rect.bottom()+18, f"{x:.1f}")
            x += self.tick_step_m

        y = 0.0
        while y <= self.map_height_m + 1e-9:
            py = self._wy_to_py(y, rect)
            painter.drawLine(rect.left()-5, py, rect.left(), py)
            painter.drawText(rect.left()-30, py+4, f"{y:.1f}")
            y += self.tick_step_m

        # 트레일 그리기
        if self.show_trails:
            robot_colors = [
                QColor(255, 156, 74),   # 로봇 1 - 주황색
                QColor(100, 255, 100),  # 로봇 2 - 초록색  
                QColor(255, 100, 100),  # 로봇 3 - 빨간색
                QColor(100, 255, 255),  # 로봇 4 - 시안
                QColor(255, 255, 100),  # 로봇 5 - 노란색
                QColor(255, 100, 255)  # 로봇 6 - 마젠타
                
            ]
            
            for rid, pts in self.trails.items():
                if len(pts) < 2:
                    continue
                base = robot_colors[rid % len(robot_colors)]
                n = len(pts)
                for i in range(1, n):
                    a = int(60 + 180 * (i / (n-1)))
                    pen = QPen(QColor(base.red(), base.green(), base.blue(), a), 2)
                    painter.setPen(pen)
                    x0, y0 = pts[i-1]
                    x1, y1 = pts[i]
                    painter.drawLine(self._wx_to_px(x0, rect), self._wy_to_py(y0, rect),
                                     self._wx_to_px(x1, rect), self._wy_to_py(y1, rect))

        # 로봇 그리기
        robot_colors = [
            QColor(255, 156, 74),   # 로봇 1 - 주황색
            QColor(100, 255, 100),  # 로봇 2 - 초록색
            QColor(255, 100, 100),  # 로봇 3 - 빨간색
            QColor(100, 255, 255),  # 로봇 4 - 시안
            QColor(255, 255, 100),  # 로봇 5 - 노란색
            QColor(255, 100, 255)   # 로봇 6 - 마젠타
            
        ]
        
        for rid, pos in self.robot_positions.items():
            color = robot_colors[rid % len(robot_colors)]
            px = self._wx_to_px(pos['x'], rect)
            py = self._wy_to_py(pos['y'], rect)

            # Start 점
            if rid in self.starts:
                sx0 = self._wx_to_px(self.starts[rid][0], rect)
                sy0 = self._wy_to_py(self.starts[rid][1], rect)
                painter.setPen(QPen(QColor(0,0,0), 3))
                painter.setBrush(QBrush(QColor(240,240,240)))
                painter.drawEllipse(sx0-6, sy0-6, 12, 12)

            # Current
            if rid == self.selected_robot_id:
                painter.setPen(QPen(QColor(255, 255, 255), 4))
            else:
                painter.setPen(QPen(QColor(255,144,64), 3))
            
            painter.setBrush(QBrush(color))
            painter.drawEllipse(px-8, py-8, 16, 16)

            # Heading arrow
            yaw = pos['yaw']
            L = 22
            head_len = 10
            head_w = 10

            dx = math.cos(yaw)
            dy = -math.sin(yaw)

            sx = int(px + (L - head_len) * dx)
            sy = int(py + (L - head_len) * dy)

            tx = int(px + L * dx)
            ty = int(py + L * dy)

            perp_x = -dy
            perp_y = dx

            lx = int(sx + (head_w/2.0) * perp_x)
            ly = int(sy + (head_w/2.0) * perp_y)
            rx = int(sx - (head_w/2.0) * perp_x)
            ry = int(sy - (head_w/2.0) * perp_y)

            arrow_color = QColor(255, 144, 64)
            painter.setPen(QPen(arrow_color, 3))
            painter.drawLine(px, py, sx, sy)

            painter.setPen(Qt.NoPen)
            painter.setBrush(QBrush(arrow_color))
            polygon = QPolygon([QPoint(tx, ty), QPoint(lx, ly), QPoint(rx, ry)])
            painter.drawPolygon(polygon)

        # 목표 위치 그리기
        if self.target_pose and self.target_pose['robot_id'] == self.selected_robot_id:
            target_x, target_y = self.target_pose['x'], self.target_pose['y']
            target_px = self._wx_to_px(target_x, rect)
            target_py = self._wy_to_py(target_y, rect)
            
            painter.setPen(QPen(QColor(255, 0, 0), 2))
            painter.setBrush(QBrush(QColor(255, 0, 0, 100)))
            painter.drawEllipse(target_px - 6, target_py - 6, 12, 12)
            
            target_yaw = self.target_pose['yaw']
            arrow_length = 15
            arrow_end_x = target_px + arrow_length * math.cos(target_yaw)
            arrow_end_y = target_py - arrow_length * math.sin(target_yaw)
            painter.setPen(QPen(QColor(255, 0, 0), 2))
            painter.drawLine(target_px, target_py, int(arrow_end_x), int(arrow_end_y))

        # 드래그 중인 화살표 표시
        if self.drag_start and self.drag_current and self.selected_robot_id is not None:
            start_px = self._wx_to_px(self.drag_start[0], rect)
            start_py = self._wy_to_py(self.drag_start[1], rect)
            current_px = self._wx_to_px(self.drag_current[0], rect)
            current_py = self._wy_to_py(self.drag_current[1], rect)
            
            painter.setPen(QPen(QColor(255, 165, 0), 3))
            painter.drawLine(start_px, start_py, current_px, current_py)
            
            painter.setBrush(QBrush(QColor(255, 165, 0)))
            painter.drawEllipse(current_px - 5, current_py - 5, 10, 10)

        painter.end()
        self.setPixmap(canvas)

    def update_robot_positions(self, robot_states: dict):
        """외부(ROS)에서 넘어온 상태로 UI 반영 + 트레일 축적"""
        self.robot_positions = robot_states
        if self.show_trails and robot_states:
            self._accumulate_trail(robot_states)
        self.update_map()

# ==============================================================================
# 3. Integrated ROS Thread
# ==============================================================================

# Flask 앱 설정
app = Flask(__name__)
frame_lock = threading.Lock()
flask_frame = None

def generate_frames():
    global flask_frame
    while True:
        with frame_lock:
            if flask_frame is None:
                time.sleep(0.1)
                continue
        current_frame = flask_frame

        yield (b'--frame\r\n'
        b'Content-Type: image/jpeg\r\n\r\n' + current_frame + b'\r\n')

@app.route('/stream')
def video_feed():
    return Response(generate_frames(),
    mimetype='multipart/x-mixed-replace; boundary=frame')

class IntegratedRosThread(QThread):
    # GUI 업데이트 시그널들
    state_update = pyqtSignal(int, str)
    angle_update = pyqtSignal(int, float)
    dist_update = pyqtSignal(int, float)
    img_update = pyqtSignal(object)
    robot_states_update = pyqtSignal(dict)
    performance_update = pyqtSignal(dict)
    camera_pose_update = pyqtSignal(int, object)

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.running = True
        
        # ArUco 관련 초기화
        self.setup_aruco()
        
        # 파라미터 서비스 클라이언트들
        self.param_clients = {}
        
        # 로봇별 구독자들
        self.subscribers = {}
        
        # ArUco 관련 퍼블리셔들
        self.pose_publishers = {}
        # === 수정된 부분: 단일 퍼블리셔로 생성 ===
        self.aruco_poses_array_publisher = self.node.create_publisher(
            ArucoPoseArray, '/aruco_pose_array', 10)
        
        self.odom_publishers = {}
        self.goal_publishers = {}
        self.robot_states = {}
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self.node)
        
        # 필터 계수들
        self.velocity_alpha = 0.3
        self.position_alpha = 0.7
        self.yaw_alpha = 0.6
        self.min_motion_m = 0.02
        
        self.odom_frame = "odom"
        self.map_frame = "map"

    def setup_aruco(self):
        """ArUco 검출기 초기화"""
        try:
            # 경로 후보들
            candidates = []
            env_dir = os.environ.get('ARUCO_CALIB_DIR', '').strip()
            if env_dir: 
                candidates.append(Path(env_dir))
            candidates.append(Path('/home/addinnedu/ROS2_Jazzy_Study/src/aruco_marker_pkg/include'))
            candidates.append(Path(__file__).resolve().parent / 'include')

            cm_path = dc_path = None
            for d in candidates:
                cm = d / 'camera_matrix.npy'
                dc = d / 'dist_coeffs.npy'
                if cm.exists() and dc.exists():
                    cm_path, dc_path = cm, dc
                    break
            
            if cm_path is None:
                raise FileNotFoundError(f"camera_matrix.npy/dist_coeffs.npy not found in: {', '.join(map(str, candidates))}")

            camera_matrix = np.load(str(cm_path))
            dist_coeffs = np.load(str(dc_path))
            self.detector = ArucoDetector(camera_matrix, dist_coeffs)
            
            # 카메라 초기화
            self.cap = cv2.VideoCapture(2, cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1200)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
            self.cap.set(cv2.CAP_PROP_FPS, 30)

            if not self.cap.isOpened():
                raise RuntimeError(f"Failed to open camera device")

            width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            fps = self.cap.get(cv2.CAP_PROP_FPS)
            self.node.get_logger().info(f"Camera: {width}x{height} @ {fps}fps")
            
        except Exception as e:
            self.node.get_logger().error(f"ArUco setup failed: {e}")
            self.cap = None
            self.detector = None

    def setup_robot_subscribers(self, robot_id):
        """특정 로봇의 구독자들 설정"""
        if robot_id not in self.subscribers:
            self.subscribers[robot_id] = {}
            
            self.subscribers[robot_id]['state'] = self.node.create_subscription(
                String, f'/robot{robot_id}/state', 
                lambda msg, rid=robot_id: self.state_update.emit(rid, msg.data), 10)
            
            self.subscribers[robot_id]['angle_error'] = self.node.create_subscription(
                Float64, f'/robot{robot_id}/angle_error',
                lambda msg, rid=robot_id: self.angle_update.emit(rid, msg.data), 10)
            
            self.subscribers[robot_id]['distance_error'] = self.node.create_subscription(
                Float64, f'/robot{robot_id}/distance_error',
                lambda msg, rid=robot_id: self.dist_update.emit(rid, msg.data), 10)
            
            self.subscribers[robot_id]['camera_pose'] = self.node.create_subscription(
                PoseStamped, f'/robot{robot_id}/camera_pose',
                lambda msg, rid=robot_id: self.camera_pose_update.emit(rid, msg), 10)

    def setup_robot_publishers(self, robot_id):
        """특정 로봇의 퍼블리셔들 설정"""
        if robot_id not in self.pose_publishers:
            self.pose_publishers[robot_id] = self.node.create_publisher(
                PoseStamped, f"/robot{robot_id}/pose", 10)
        
        # === 제거된 부분: ArucoPoseArray 관련 코드 삭제됨 ===
        # if robot_id not in self.aruco_poses_array_publisher:    
        #     self.aruco_poses_array_publisher = self.node.create_publisher(
        #         ArucoPoseArray, '/aruco_pose_array', 10)
        
        if robot_id not in self.odom_publishers:
            self.odom_publishers[robot_id] = self.node.create_publisher(
                Odometry, f"/odom_{robot_id}", 10)
        
        if robot_id not in self.goal_publishers:
            self.goal_publishers[robot_id] = self.node.create_publisher(
                PoseStamped, f"/robot{robot_id}/goal_pose", 10)

    def setup_robot_param_client(self, robot_id):
        """특정 로봇의 파라미터 클라이언트 설정"""
        if robot_id not in self.param_clients:
            self.param_clients[robot_id] = self.node.create_client(
                SetParameters, f'/robot{robot_id}/simple_robot_goal_controller/set_parameters')

    def publish_target_pose(self, robot_id, pose_msg):
        """특정 로봇에게 목표 위치 발행"""
        if robot_id in self.goal_publishers:
            self.goal_publishers[robot_id].publish(pose_msg)
            self.node.get_logger().info(
                f"Published goal for robot {robot_id}: x={pose_msg.pose.position.x:.2f}, y={pose_msg.pose.position.y:.2f}")

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

        # 데드밴드
        dx_try = x_filtered - state.x
        dy_try = y_filtered - state.y
        if (dx_try*dx_try + dy_try*dy_try) < (self.min_motion_m*self.min_motion_m):
            x_applied, y_applied = state.x, state.y
            vx_new = 0.0
            vy_new = 0.0
        else:
            x_applied, y_applied = x_filtered, y_filtered
            vx_new = (x_applied - state.last_x) / dt
            vy_new = (y_applied - state.last_y) / dt

        dyaw = normalize_angle(yaw_filtered - state.last_yaw)
        vyaw_new = dyaw / dt

        # 속도 필터링
        state.vx = self.velocity_alpha * state.vx + (1 - self.velocity_alpha) * vx_new
        state.vy = self.velocity_alpha * state.vy + (1 - self.velocity_alpha) * vy_new
        state.vyaw = self.velocity_alpha * state.vyaw + (1 - self.velocity_alpha) * vyaw_new

        # 상태 업데이트
        state.last_x = state.x
        state.last_y = state.y
        state.last_yaw = state.yaw
        
        state.x = x_applied
        state.y = y_applied
        state.yaw = yaw_filtered
        state.last_time = current_time

        # 궤적 업데이트
        state.trajectory.append((x_applied, y_applied))
        if len(state.trajectory) > 500:
            state.trajectory.pop(0)

    def create_aruco_poses_array_message(self, detected_markers):
        """여러 ArUco 마커들의 정보를 담은 ArucoPoseArray 메시지 생성"""
        poses_array = ArucoPoseArray()
        
        # Header 설정
        poses_array.header = Header()
        poses_array.header.stamp = self.node.get_clock().now().to_msg()
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
        
    def create_odometry_message(self, marker_id, current_time):
        """Odometry 메시지 생성"""
        if marker_id not in self.robot_states:
            return None

        state = self.robot_states[marker_id]
        if not state.initialized:
            return None

        odom = Odometry()
        odom.header.stamp = self.node.get_clock().now().to_msg()
        odom.header.frame_id = f"odom_{marker_id}"
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
                    0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 1e6,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.05]

        odom.pose.covariance = pose_cov
        odom.twist.covariance = twist_cov

        return odom

    def run(self):
        """메인 실행 루프"""
        self.node.get_logger().info("통합 ROS Thread 시작")
        
        # Flask 스트리밍 서버 시작
        threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000, threaded=True), daemon=True).start()

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
        
        # Flask 스트리밍용 프레임 저장
        global flask_frame
        _, buffer = cv2.imencode('.jpg', frame)
        with frame_lock:
            flask_frame = buffer.tobytes()

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

            # Publishers 및 Subscribers 생성
            self.setup_robot_publishers(marker_id)
            self.setup_robot_subscribers(marker_id)
            self.setup_robot_param_client(marker_id)

            # 메시지 발행
            state = self.robot_states[marker_id]
            
            # PoseStamped 발행 (기존 코드)
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

            # Odometry 발행 (기존 코드)
            odom = self.create_odometry_message(marker_id, current_time)
            if odom is not None:
                self.odom_publishers[marker_id].publish(odom)

        # === 새로 추가된 부분: ArucoPoseArray 발행 ===
        # 모든 검출된 마커들을 배열로 발행
        if detected:
            aruco_poses_array_msg = self.create_aruco_poses_array_message(detected)
            self.aruco_poses_array_publisher.publish(aruco_poses_array_msg)

        processing_end = time.time()
        processing_time_ms = (processing_end - processing_start) * 1000

        # GUI로 이미지 전송 (BGR -> RGB 변환)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.img_update.emit(rgb_frame)

        # 로봇 상태 정보 GUI로 전송
        robot_info = {}
        for robot_id, state in self.robot_states.items():
            if state.initialized and (current_time - state.last_seen) < 1.0:
                robot_info[robot_id] = {
                    'x': state.x,
                    'y': state.y,
                    'yaw': state.yaw,
                    'vx': state.vx,
                    'vy': state.vy,
                    'vyaw': state.vyaw,
                    'last_seen': state.last_seen,
                    'current_state': state.current_state,
                    'angle_error': state.angle_error,
                    'distance_error': state.distance_error,
                    'trajectory': state.trajectory[-50:] if len(state.trajectory) > 50 else state.trajectory
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

    def set_robot_parameter(self, robot_id, name, value):
        """특정 로봇의 파라미터 설정"""
        if robot_id in self.param_clients:
            client = self.param_clients[robot_id]
            if not client.wait_for_service(timeout_sec=0.1):
                return
            try:
                req = SetParameters.Request()
                if isinstance(value, bool):
                    param = Parameter(name=name,
                                      value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                                           bool_value=value))
                else:
                    param = Parameter(name=name,
                                      value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                                           double_value=value))
                req.parameters.append(param)
                client.call_async(req)
            except Exception as e:
                self.node.get_logger().error(f"Set robot {robot_id} parameter error: {e}")

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

# ==============================================================================
# 4. Main Window GUI
# ==============================================================================

class IntegratedMainWindow(QMainWindow):
    def __init__(self, ros_thread, node):
        super().__init__()
        print('start main window')
        self.setWindowTitle('통합 AMR 제어 및 모니터링 시스템')
        self.resize(1800, 1200)

        # 중앙 위젯
        central = QWidget()
        self.setCentralWidget(central)
        
        # 메인 레이아웃 (2x2 그리드)
        main_layout = QGridLayout(central)
        main_layout.setSpacing(10)

        # ROS 스레드 및 노드 저장
        self.ros_thread = ros_thread
        self.ros_node = node
        
        # 현재 선택된 로봇 ID
        self.selected_robot_id = None
        
        # 설정 파일 경로
        self.config_path = os.path.join(os.path.dirname(__file__), 'amr_config.yaml')
        
        # 상태 리스트
        self.state_list = ["IDLE", "RotateToGoal", "MoveToGoal", "RotateToFinal", "GoalReached"]
        
        # 도메인 브리지용 목표 발행자들 추가
        self.goal_bridge_publishers = {}
        self.setup_goal_bridge_publishers()
        
        # 1사분면: AMR 맵
        self.setup_map_section(main_layout)
        
        # 2사분면: 라이브 카메라
        self.setup_camera_section(main_layout)
        
        # 3사분면: 제어 파라미터
        self.setup_control_section(main_layout)
        
        # 4사분면: 상태 모니터링
        self.setup_status_section(main_layout)

        # 그리드 레이아웃 비율 설정
        main_layout.setRowStretch(0, 1)
        main_layout.setRowStretch(1, 1)
        main_layout.setColumnStretch(0, 1)
        main_layout.setColumnStretch(1, 1)

        # ROS 시그널 연결
        self.connect_ros_signals()
        
        # 설정 로드
        self.load_config()
        
        # GUI 업데이트 타이머
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_gui_elements)
        self.update_timer.start(100)  # 100ms
        
        # FPS 계산용
        self.fps_counter = 0
        self.fps_timer = QTimer()
        self.fps_timer.timeout.connect(self.update_fps_display)
        self.fps_timer.start(1000)

        # 긴급 정지용 퍼블리셔들 추가
        self.emergency_publishers = {}
        self.setup_emergency_publishers()
    def setup_emergency_publishers(self):
        """긴급 정지용 퍼블리셔들 설정"""
        # 각 로봇별 긴급 정지 토픽 생성
        for robot_id in range(1, 7):  # 로봇 1-6번
            # 속도 명령 토픽 (즉시 정지용)
            self.emergency_publishers[f'cmd_vel_{robot_id}'] = self.ros_node.create_publisher(
                Twist, f'/robot{robot_id}/cmd_vel', 10)
            
            # 긴급 정지 신호 토픽
            self.emergency_publishers[f'emergency_{robot_id}'] = self.ros_node.create_publisher(
                String, f'/robot{robot_id}/emergency_stop', 10)


    def setup_goal_bridge_publishers(self):
        """도메인 브리지용 목표 발행자들 설정"""
        self.goal_bridge_publishers[1] = self.ros_node.create_publisher(
            PoseStamped, '/goalpose1', 10)
        self.goal_bridge_publishers[2] = self.ros_node.create_publisher(
            PoseStamped, '/goalpose2', 10)
        self.goal_bridge_publishers[3] = self.ros_node.create_publisher(
            PoseStamped, '/goalpose3', 10)

    def setup_map_section(self, main_layout):
        """맵 섹션 설정"""
        map_group = QGroupBox('AMR 맵 + 로봇 제어')
        map_layout = QVBoxLayout()
        
        # 로봇 선택 영역
        robot_select_layout = QHBoxLayout()
        robot_select_layout.addWidget(QLabel("제어할 로봇 선택:"))
        
        self.robot_selection_group = QButtonGroup()
        self.robot_radio_buttons = {}
        
        robot_select_layout.addStretch()
        map_layout.addLayout(robot_select_layout)
        
        # 커스텀 맵 생성 (20x10 + 테두리)
        cols, rows = 20, 10
        custom_map = [[1]*cols] + [[1]+[0]*(cols-2)+[1] for _ in range(rows-2)] + [[1]*cols]
        
        # 맵 위젯
        self.map_widget = MapWidget(custom_map)
        map_layout.addWidget(self.map_widget, alignment=Qt.AlignCenter)
        
        # 맵 정보 표시
        map_info_layout = QHBoxLayout()
        self.map_info_label = QLabel("맵 크기: 2.0m x 1.0m | 활성 로봇: 0 | 선택된 로봇: None")
        self.clear_trajectory_btn = QPushButton("궤적 지우기")
        self.clear_trajectory_btn.clicked.connect(self.clear_trajectories)
        map_info_layout.addWidget(self.map_info_label)
        map_info_layout.addWidget(self.clear_trajectory_btn)
        map_layout.addLayout(map_info_layout)
        
        map_group.setLayout(map_layout)
        main_layout.addWidget(map_group, 0, 0)

    def setup_camera_section(self, main_layout):
        """카메라 섹션 설정"""
        cam_group = QGroupBox('Live Camera + ArUco Detection')
        cam_layout = QVBoxLayout()
        
        self.lbl_cam = QLabel()
        self.lbl_cam.setFixedSize(800, 400)
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

    def setup_control_section(self, main_layout):
        """제어 파라미터 섹션 설정"""
        control_group = QGroupBox('제어 파라미터')
        control_layout = QVBoxLayout()
        
        # 파라미터 탭
        param_tabs = QTabWidget()
        
        # PID 제어 탭
        self.setup_pid_tab(param_tabs)
        
        # Goal Mover 탭
        self.setup_goal_mover_tab(param_tabs)
        
        # ArUco 설정 탭
        self.setup_aruco_tab(param_tabs)
        
        control_layout.addWidget(param_tabs)
        control_group.setLayout(control_layout)
        main_layout.addWidget(control_group, 1, 0)

    def setup_pid_tab(self, param_tabs):
        """PID 제어 탭 설정"""
        pid_widget = QWidget()
        pid_layout = QGridLayout()
        
        self.pid_spinboxes = {}
        
        # 유클리드 거리 사용 옵션
        self.euclidean_checkbox = QCheckBox("유클리드 거리 사용")
        self.euclidean_checkbox.setChecked(True)
        self.euclidean_checkbox.stateChanged.connect(self.update_distance_mode)
        pid_layout.addWidget(self.euclidean_checkbox, 0, 0, 1, 3)

        # PID 파라미터들
        params = [
            (('tolerances', 'angle'), "각도 허용오차:", 0.1, -10.0, 10.0, 0.001),
            (('tolerances', 'distance'), "거리 허용오차:", 0.05, -10.0, 10.0, 0.001),
            (('angular', 'P'), "Angular P:", 1.0, -10.0, 10.0, 0.001),
            (('angular', 'I'), "Angular I:", 0.0, -10.0, 10.0, 0.001),
            (('angular', 'D'), "Angular D:", 0.1, -10.0, 10.0, 0.001),
            (('angular', 'max_state'), "Angular Max:", 2.0, -10.0, 10.0, 0.001),
            (('angular', 'min_state'), "Angular Min:", -2.0, -10.0, 10.0, 0.001),
            (('linear', 'P'), "Linear P:", 0.5, -10.0, 10.0, 0.001),
            (('linear', 'I'), "Linear I:", 0.0, -10.0, 10.0, 0.001),
            (('linear', 'D'), "Linear D:", 0.1, -10.0, 10.0, 0.001),
            (('linear', 'max_state'), "Linear Max:", 1.0, -10.0, 10.0, 0.001),
            (('linear', 'min_state'), "Linear Min:", -1.0, -10.0, 10.0, 0.001)
        ]

        for i, ((group, param), label_text, default_val, min_val, max_val, step) in enumerate(params):
            label = QLabel(label_text)
            spinbox = QDoubleSpinBox()
            spinbox.setRange(min_val, max_val)
            spinbox.setDecimals(3)
            spinbox.setSingleStep(step)
            spinbox.setValue(default_val)
            spinbox.valueChanged.connect(lambda v, g=group, p=param: self.update_pid_parameter(g, p, v))
            
            self.pid_spinboxes[(group, param)] = spinbox
            pid_layout.addWidget(label, i + 1, 0)
            pid_layout.addWidget(spinbox, i + 1, 1)
            
            # 현재 값 표시
            val_label = QLabel(f"{default_val:.3f}")
            pid_layout.addWidget(val_label, i + 1, 2)
            spinbox.valueChanged.connect(lambda v, lbl=val_label: lbl.setText(f"{v:.3f}"))

        # 제어 버튼들
        self.save_pid_button = QPushButton("PID 설정 저장")
        self.load_pid_button = QPushButton("PID 설정 로드")
        self.save_pid_button.clicked.connect(self.save_config)
        self.load_pid_button.clicked.connect(self.load_config)
        
        pid_layout.addWidget(self.save_pid_button, len(params) + 1, 0)
        pid_layout.addWidget(self.load_pid_button, len(params) + 1, 1)
        
        pid_widget.setLayout(pid_layout)
        param_tabs.addTab(pid_widget, "PID Control")

    def setup_goal_mover_tab(self, param_tabs):
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
            sb.valueChanged.connect(lambda v, n=name: self.update_goal_mover_param(n, v))
            goal_layout.addWidget(sb, i, 1)
            self.goal_spin[name] = sb
            
            # 현재 값 표시
            val_label = QLabel(f"{defv:.3f}")
            goal_layout.addWidget(val_label, i, 2)
            sb.valueChanged.connect(lambda v, lbl=val_label: lbl.setText(f"{v:.3f}"))
        
        # 프리셋 버튼들
        self._add_preset_buttons(goal_layout, len(goal_params))
        
        goal_widget.setLayout(goal_layout)
        param_tabs.addTab(goal_widget, "Goal Mover")

    def setup_aruco_tab(self, param_tabs):
        """ArUco 설정 탭"""
        aruco_widget = QWidget()
        aruco_layout = QGridLayout()
        
        # ArUco 필터 설정
        filter_params = [
            ('Position Alpha', 'position_alpha', 0.7, 0.0, 1.0, 0.1),
            ('Velocity Alpha', 'velocity_alpha', 0.3, 0.0, 1.0, 0.1),
            ('Yaw Alpha', 'yaw_alpha', 0.6, 0.0, 1.0, 0.1),
            ('Min Motion (m)', 'min_motion_m', 0.02, 0.0, 0.2, 0.005)
        ]
        
        self.aruco_spin = {}
        for i, (lbl, name, defv, minv, maxv, step) in enumerate(filter_params):
            aruco_layout.addWidget(QLabel(lbl), i, 0)
            sb = QDoubleSpinBox()
            sb.setRange(minv, maxv)
            sb.setDecimals(3)
            sb.setSingleStep(step)
            sb.setValue(defv)
            sb.valueChanged.connect(lambda v, n=name: self.update_aruco_param(n, v))
            aruco_layout.addWidget(sb, i, 1)
            self.aruco_spin[name] = sb
            
            val_label = QLabel(f"{defv:.3f}")
            aruco_layout.addWidget(val_label, i, 2)
            sb.valueChanged.connect(lambda v, lbl=val_label: lbl.setText(f"{v:.3f}"))
        
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

    def setup_status_section(self, main_layout):
        """상태 모니터링 섹션 설정"""
        status_group = QGroupBox('로봇 상태 모니터링')
        status_layout = QVBoxLayout()
        
        # 상태 흐름 다이어그램
        state_flow_group = QGroupBox('상태 흐름')
        state_flow_layout = QVBoxLayout()
        self.figure_state = Figure(figsize=(6, 4))
        self.canvas_state = FigureCanvas(self.figure_state)
        self.ax_state = self.figure_state.add_subplot(111)
        state_flow_layout.addWidget(self.canvas_state)
        state_flow_group.setLayout(state_flow_layout)
        status_layout.addWidget(state_flow_group)
        
        # 로봇 상태 표시 (스크롤 가능)
        robot_status_group = QGroupBox('개별 로봇 상태')
        robot_status_layout = QVBoxLayout()
        
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
        
        # 로그 텍스트
        log_group = QGroupBox("시스템 로그")
        log_layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setMaximumHeight(100)
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)
        status_layout.addWidget(log_group)
        
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

    def _add_preset_buttons(self, layout, start_row):
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
            btn.clicked.connect(lambda checked, p=params: self._apply_preset(p))
            layout.addWidget(btn, start_row, i + 1)

    def _apply_preset(self, preset_params):
        """프리셋 파라미터들을 적용"""
        for param_name, value in preset_params.items():
            if param_name in self.goal_spin:
                self.goal_spin[param_name].setValue(value)
                self.update_goal_mover_param(param_name, value)

    def connect_ros_signals(self):
        """ROS 시그널들 연결"""
        self.ros_thread.state_update.connect(self.update_robot_state)
        self.ros_thread.angle_update.connect(self.update_robot_angle_error)
        self.ros_thread.dist_update.connect(self.update_robot_distance_error)
        self.ros_thread.img_update.connect(self._update_camera)
        self.ros_thread.robot_states_update.connect(self._update_robot_states)
        self.ros_thread.performance_update.connect(self._update_performance)
        self.ros_thread.camera_pose_update.connect(self.update_robot_trajectory)

    # 업데이트 메서드들
    def update_robot_state(self, robot_id, state):
        """로봇 상태 업데이트"""
        if robot_id in self.ros_thread.robot_states:
            self.ros_thread.robot_states[robot_id].current_state = state
        self.add_log(f"로봇 {robot_id} 상태: {state}")

    def update_robot_angle_error(self, robot_id, angle_error):
        """로봇 각도 오차 업데이트"""
        if robot_id in self.ros_thread.robot_states:
            self.ros_thread.robot_states[robot_id].angle_error = angle_error

    def update_robot_distance_error(self, robot_id, distance_error):
        """로봇 거리 오차 업데이트"""
        if robot_id in self.ros_thread.robot_states:
            self.ros_thread.robot_states[robot_id].distance_error = distance_error

    def update_robot_trajectory(self, robot_id, pose):
        """로봇 궤적 업데이트"""
        if robot_id in self.ros_thread.robot_states:
            x = pose.pose.position.x
            y = pose.pose.position.y
            self.ros_thread.robot_states[robot_id].trajectory.append((x, y))
            if len(self.ros_thread.robot_states[robot_id].trajectory) > 500:
                self.ros_thread.robot_states[robot_id].trajectory.pop(0)

    def on_goal_set(self, robot_id, target_pose):
        """맵에서 목표가 설정되었을 때 호출"""
        # 목표 메시지 생성
        target_msg = PoseStamped()
        target_msg.header.stamp = self.ros_node.get_clock().now().to_msg()
        target_msg.header.frame_id = 'map'
        target_msg.pose.position.x = float(target_pose['x'])
        target_msg.pose.position.y = float(target_pose['y'])
        target_msg.pose.position.z = 0.0
        
        # 쿼터니언으로 자세 설정
        theta = target_pose['yaw']
        target_msg.pose.orientation.z = math.sin(theta / 2.0)
        target_msg.pose.orientation.w = math.cos(theta / 2.0)

        # 도메인 브리지용 목표 발행
        if robot_id in self.goal_bridge_publishers:
            self.goal_bridge_publishers[robot_id].publish(target_msg)
            self.add_log(f"도메인 브리지로 목표 발행: /goalpose{robot_id}")
        
        # 기존 방식도 유지
        self.ros_thread.publish_target_pose(robot_id, target_msg)
        
        self.add_log(
            f"로봇 {robot_id} 새 목표: ({target_pose['x']:.2f}, {target_pose['y']:.2f}), "
            f"각도: {math.degrees(theta):.1f}°"
        )

    def on_robot_selected(self, robot_id):
        """로봇이 선택되었을 때 호출"""
        self.selected_robot_id = robot_id
        self.map_widget.set_selected_robot(robot_id)
        self.update_map_info()
        self.add_log(f"로봇 {robot_id} 선택됨")

    def update_map_info(self):
        """맵 정보 레이블 업데이트"""
        active_count = len(self.ros_thread.robot_states)
        selected_text = f"R{self.selected_robot_id}" if self.selected_robot_id is not None else "None"
        self.map_info_label.setText(f"맵 크기: 2.0m x 1.0m | 활성 로봇: {active_count} | 선택된 로봇: {selected_text}")

    def clear_trajectories(self):
        """모든 로봇의 궤적 지우기"""
        for robot_state in self.ros_thread.robot_states.values():
            robot_state.trajectory.clear()
        self.map_widget.trails.clear()
        self.map_widget.starts.clear()
        self.add_log("모든 궤적을 지웠습니다.")

    def update_distance_mode(self, state):
        """거리 계산 모드 업데이트"""
        use_euclidean = (state == Qt.Checked)
        if self.selected_robot_id is not None:
            self.ros_thread.set_robot_parameter(self.selected_robot_id, 'use_euclidean_distance', use_euclidean)
        self.add_log(f"유클리드 거리 사용: {use_euclidean}")

    def update_pid_parameter(self, group, param, value):
        """PID 파라미터 업데이트"""
        if self.selected_robot_id is not None:
            param_name = f"{group}.{param}"
            self.ros_thread.set_robot_parameter(self.selected_robot_id, param_name, value)
            self.add_log(f"로봇 {self.selected_robot_id} {param_name}: {value:.3f}")

    def update_goal_mover_param(self, name, value):
        """Goal Mover 파라미터 업데이트"""
        if self.selected_robot_id is not None:
            self.ros_thread.set_robot_parameter(self.selected_robot_id, name, value)
            self.add_log(f"로봇 {self.selected_robot_id} {name}: {value:.3f}")

    def update_aruco_param(self, name, value):
        """ArUco 파라미터 업데이트"""
        if hasattr(self.ros_thread, name):
            setattr(self.ros_thread, name, value)
            self.add_log(f"ArUco {name}: {value:.3f}")

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
        
        # 로봇 선택 버튼 업데이트
        self.update_robot_selection_buttons(robot_states.keys())
        
        # 맵 정보 업데이트
        self.update_map_info()
        
        # 개별 로봇 상태 표시 업데이트
        self.update_robot_details_display(robot_states)

    def update_robot_selection_buttons(self, active_robot_ids):
        """로봇 선택 버튼들 업데이트"""
        # 기존 버튼들 제거
        for robot_id, button in list(self.robot_radio_buttons.items()):
            if robot_id not in active_robot_ids:
                self.robot_selection_group.removeButton(button)
                button.deleteLater()
                del self.robot_radio_buttons[robot_id]
        
        # 새로운 로봇 버튼들 추가
        for robot_id in active_robot_ids:
            if robot_id not in self.robot_radio_buttons:
                button = QRadioButton(f"로봇 {robot_id}")
                button.clicked.connect(lambda checked, rid=robot_id: self.on_robot_selected(rid) if checked else None)
                self.robot_selection_group.addButton(button)
                self.robot_radio_buttons[robot_id] = button
                
                # 레이아웃에 추가
                map_group = self.centralWidget().layout().itemAtPosition(0, 0).widget()
                robot_select_layout = map_group.layout().itemAt(0).layout()
                robot_select_layout.insertWidget(robot_select_layout.count() - 1, button)
        
        # 첫 번째 로봇을 자동 선택
        if self.selected_robot_id is None and active_robot_ids:
            first_robot_id = min(active_robot_ids)
            if first_robot_id in self.robot_radio_buttons:
                self.robot_radio_buttons[first_robot_id].setChecked(True)
                self.on_robot_selected(first_robot_id)

    def update_robot_details_display(self, robot_states):
        """개별 로봇 상태 표시 업데이트"""
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
            
            # 상태 정보
            robot_layout.addWidget(QLabel('상태:'), 3, 0)
            robot_layout.addWidget(QLabel(state.get("current_state", "UNKNOWN")), 3, 1)
            
            # 오차 정보
            robot_layout.addWidget(QLabel('각도 오차:'), 4, 0)
            robot_layout.addWidget(QLabel(f'{math.degrees(state.get("angle_error", 0)):.1f}°'), 4, 1)
            
            robot_layout.addWidget(QLabel('거리 오차:'), 5, 0)
            robot_layout.addWidget(QLabel(f'{state.get("distance_error", 0):.3f} m'), 5, 1)
            
            robot_widget.setLayout(robot_layout)
            self.robot_details_layout.addWidget(robot_widget)

    def _update_performance(self, perf_info):
        """성능 정보 업데이트"""
        self.detection_label.setText(f"Detections: {perf_info['marker_count']}")
        self.detection_fps_label.setText(f"검출 FPS: {perf_info['fps']:.1f}")
        self.marker_count_label.setText(f"감지된 마커: {perf_info['marker_count']}")
        self.lbl_performance.setText(f"처리 시간: {perf_info['total_time']:.1f}ms")

    def update_gui_elements(self):
        """GUI 요소들 주기적 업데이트"""
        self.update_state_diagram()

    def update_state_diagram(self):
        """상태 다이어그램 업데이트"""
        self.ax_state.clear()
        self.ax_state.set_xlim(0, 1.5)
        self.ax_state.set_ylim(0, 1)
        self.ax_state.axis('off')
        
        if self.selected_robot_id is not None and self.selected_robot_id in self.ros_thread.robot_states:
            current_state = self.ros_thread.robot_states[self.selected_robot_id].current_state
        else:
            current_state = "IDLE"
        
        # 상태 박스 설정
        block_width, block_height, spacing = 1.2, 0.15, 0.05
        start_x = (1.5 - block_width) / 2
        n = len(self.state_list)
        total_height = n * block_height + (n - 1) * spacing
        group_bottom = 0.5 - total_height / 2
        
        for i, state in enumerate(self.state_list):
            y = group_bottom + (n - 1 - i) * (block_height + spacing)
            
            # 현재 상태에 따라 색상 변경
            if current_state == state:
                face_color = '#4CAF50'  # 초록색 (활성)
                text_color = 'white'
            else:
                face_color = '#E9ECEF'  # 회색 (비활성)
                text_color = 'black'
                
            rect = FancyBboxPatch((start_x, y), block_width, block_height, 
                                 boxstyle="round,pad=0.02", fc=face_color, ec="black", lw=1.5)
            self.ax_state.add_patch(rect)
            self.ax_state.text(start_x + block_width/2, y + block_height/2, state, 
                              ha='center', va='center', fontsize=9, color=text_color, weight='bold')
        
        # 선택된 로봇 정보 표시
        if self.selected_robot_id is not None:
            self.ax_state.text(0.75, 0.9, f"로봇 {self.selected_robot_id}", 
                              ha='center', va='center', fontsize=12, weight='bold')
        
        self.canvas_state.draw()

    def update_fps_display(self):
        """FPS 표시 업데이트"""
        fps = self.fps_counter
        self.lbl_fps.setText(f'카메라 FPS: {fps}')
        self.fps_counter = 0

    # 제어 버튼 핸들러들
    def emergency_stop(self):
        """긴급 정지 기능"""
        from geometry_msgs.msg import Twist
        
        active_robots = list(self.ros_thread.robot_states.keys())
        
        if not active_robots:
            self.add_log('긴급 정지: 활성 로봇이 없습니다')
            return
        
        stop_count = 0
        for robot_id in active_robots:
            try:
                # 즉시 속도 0으로 설정
                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                twist_msg.linear.y = 0.0
                twist_msg.angular.z = 0.0
                
                if f'cmd_vel_{robot_id}' in self.emergency_publishers:
                    self.emergency_publishers[f'cmd_vel_{robot_id}'].publish(twist_msg)
                
                # 긴급 정지 신호 전송
                emergency_msg = String()
                emergency_msg.data = "EMERGENCY_STOP"
                
                if f'emergency_{robot_id}' in self.emergency_publishers:
                    self.emergency_publishers[f'emergency_{robot_id}'].publish(emergency_msg)
                
                stop_count += 1
                
            except Exception as e:
                self.add_log(f'로봇 {robot_id} 긴급 정지 실패: {str(e)}')
        
        self.add_log(f'긴급 정지 실행: {stop_count}대 로봇 정지 명령 전송')
    # ==============================================================================
    # 3. 시각적 피드백 추가
    def show_emergency_feedback(self):
        """긴급 정지 시각적 피드백"""
        # 긴급 정지 버튼을 잠시 다른 색으로 변경
        emergency_btn = self.sender()  # 클릭된 버튼 가져오기
        original_style = emergency_btn.styleSheet()
        
        # 깜빡이는 효과
        emergency_btn.setStyleSheet(
            "background-color: darkred; color: yellow; font-weight: bold; font-size: 14px; border: 3px solid yellow;"
        )
        
        # 2초 후 원래 스타일로 복원
        QTimer.singleShot(2000, lambda: emergency_btn.setStyleSheet(original_style))

    # 4. 향상된 긴급 정지 (안전 기능 추가)
    def enhanced_emergency_stop(self):
        """향상된 긴급 정지 - 추가 안전 기능"""
        from PyQt5.QtWidgets import QMessageBox
        
        # 확인 대화상자 (선택사항)
        reply = QMessageBox.question(
            self, '긴급 정지 확인', 
            '모든 로봇을 긴급 정지하시겠습니까?',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.Yes
        )
        
        if reply == QMessageBox.Yes:
            # 기본 긴급 정지 실행
            self.emergency_stop()
            
            # 추가 안전 조치
            self.disable_robot_controls()
            self.clear_all_goals()
            
            # 시스템 상태를 긴급 모드로 변경
            self.set_emergency_mode(True)

    def disable_robot_controls(self):
        """모든 로봇 제어 비활성화"""
        # 로봇 선택 버튼 비활성화
        for button in self.robot_radio_buttons.values():
            button.setEnabled(False)
        
        # 맵 클릭 비활성화
        self.map_widget.setEnabled(False)

    def clear_all_goals(self):
        """모든 로봇의 목표 위치 초기화"""
        for robot_id in self.ros_thread.robot_states.keys():
            if robot_id in self.ros_thread.goal_publishers:
                # 빈 목표 전송하여 기존 목표 취소
                empty_goal = PoseStamped()
                empty_goal.header.stamp = self.ros_node.get_clock().now().to_msg()
                empty_goal.header.frame_id = 'map'
                self.ros_thread.goal_publishers[robot_id].publish(empty_goal)

    def set_emergency_mode(self, emergency_mode):
        """시스템을 긴급 모드로 설정"""
        if emergency_mode:
            # UI를 긴급 모드 색상으로 변경
            self.setStyleSheet("QMainWindow { background-color: #2c1810; }")
            self.add_log('=== 긴급 모드 활성화 ===')
        else:
            # 일반 모드로 복원
            self.setStyleSheet("")
            self.add_log('=== 일반 모드 복원 ===')

    # 5. 긴급 모드 해제 기능
    def reset_emergency_mode(self):
        """긴급 모드 해제 및 시스템 복원"""
        # 제어 기능 재활성화
        for button in self.robot_radio_buttons.values():
            button.setEnabled(True)
        
        self.map_widget.setEnabled(True)
        
        # 긴급 모드 해제
        self.set_emergency_mode(False)
        
        self.add_log('긴급 모드 해제: 시스템 정상 작동 재개')

    # 6. 시스템 리셋에 긴급 모드 해제 추가
    def system_reset(self):
        """시스템 리셋 기능 (긴급 모드 해제 포함)"""
        # 기존 리셋 기능
        for robot_state in self.ros_thread.robot_states.values():
            robot_state.current_state = "IDLE"
            robot_state.trajectory.clear()
        
        self.map_widget.trails.clear()
        self.map_widget.starts.clear()
        self.selected_robot_id = None
        self.map_widget.set_selected_robot(None)
        
        for button in self.robot_radio_buttons.values():
            button.setChecked(False)
        
        # 긴급 모드 해제
        self.reset_emergency_mode()
        
        self.add_log('시스템 리셋 수행됨 (긴급 모드 해제 포함)')

    def save_parameters(self):
        """파라미터 저장 기능"""
        self.save_config()
        self.add_log('파라미터 저장됨')

    def load_parameters(self):
        """파라미터 로드 기능"""
        self.load_config()
        self.add_log('파라미터 로드됨')

    # 설정 파일 관리
    def load_config(self):
        """YAML 설정 파일에서 파라미터 로드"""
        try:
            if os.path.exists(self.config_path):
                with open(self.config_path, 'r') as f:
                    config = yaml.safe_load(f)
                
                # PID 스핀박스에 값 설정
                for (group, param), spinbox in self.pid_spinboxes.items():
                    if group in config and param in config[group]:
                        spinbox.setValue(float(config[group][param]))
                
                # Goal Mover 설정
                if 'goal_mover' in config:
                    for param_name, spinbox in self.goal_spin.items():
                        if param_name in config['goal_mover']:
                            spinbox.setValue(float(config['goal_mover'][param_name]))
                
                # ArUco 설정
                if 'aruco' in config:
                    for param_name, spinbox in self.aruco_spin.items():
                        if param_name in config['aruco']:
                            spinbox.setValue(float(config['aruco'][param_name]))
                
                # 유클리드 거리 설정
                if 'use_euclidean_distance' in config:
                    self.euclidean_checkbox.setChecked(config['use_euclidean_distance'])
                
                self.add_log("YAML 설정을 GUI에 로드했습니다.")
            else:
                self.add_log("설정 파일이 없습니다. 기본값을 사용합니다.")
        except Exception as e:
            self.add_log(f"YAML 설정 파일 로드 실패: {e}")

    def save_config(self):
        """현재 GUI 설정을 YAML 파일에 저장"""
        try:
            config = {}
            
            # PID 설정
            for (group, param), spinbox in self.pid_spinboxes.items():
                if group not in config:
                    config[group] = {}
                config[group][param] = spinbox.value()
            
            # Goal Mover 설정
            config['goal_mover'] = {}
            for param_name, spinbox in self.goal_spin.items():
                config['goal_mover'][param_name] = spinbox.value()
            
            # ArUco 설정
            config['aruco'] = {}
            for param_name, spinbox in self.aruco_spin.items():
                config['aruco'][param_name] = spinbox.value()
            
            # 유클리드 거리 옵션 저장
            config['use_euclidean_distance'] = self.euclidean_checkbox.isChecked()
            
            # 파일에 저장
            with open(self.config_path, 'w') as f:
                yaml.dump(config, f, default_flow_style=False, sort_keys=False)
            
            self.add_log(f"설정을 '{self.config_path}'에 저장했습니다.")
        except Exception as e:
            self.add_log(f"YAML 설정 파일 저장 실패: {e}")

    def add_log(self, message):
        """로그 메시지 추가"""
        self.log_text.append(f"[{self.get_current_time()}] {message}")
        # 자동으로 맨 아래로 스크롤
        self.log_text.verticalScrollBar().setValue(
            self.log_text.verticalScrollBar().maximum())

    def get_current_time(self):
        """현재 시간 문자열 반환"""
        return datetime.now().strftime("%H:%M:%S")

    def closeEvent(self, event):
        """창 닫기 이벤트"""
        self.ros_thread.stop()
        self.ros_thread.cleanup()
        self.ros_thread.quit()
        self.ros_thread.wait()
        event.accept()

# ==============================================================================
# 5. Main Function
# ==============================================================================

def main():
    """메인 실행 함수"""
    # ROS2 초기화
    rclpy.init()
    node = Node('integrated_amr_monitor')
    
    # 통합 ROS 스레드 생성
    ros_thread = IntegratedRosThread(node)
    
    # Qt 애플리케이션 시작
    app = QApplication(sys.argv)
    win = IntegratedMainWindow(ros_thread, node)
    win.show()
    
    # ROS 스레드 시작
    ros_thread.start()
    
    try:
        # Qt 이벤트 루프 실행
        rc = app.exec_()
    except KeyboardInterrupt:
        print("사용자에 의해 중단됨")
        rc = 0
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
