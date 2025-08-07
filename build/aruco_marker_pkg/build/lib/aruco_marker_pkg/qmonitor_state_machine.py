#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
이 스크립트는 robot_state_machine.py 노드의 상태를 모니터링하고, 제어 파라미터를
실시간으로 조절할 수 있는 PyQt5 기반의 종합적인 GUI 애플리케이션입니다.

주요 기능:
- Matplotlib 기반 실시간 Pose 시각화: 로봇의 현재 위치, 궤적, 목표 지점을 2D 격자 위에 그래픽으로 표시합니다.
- 클릭 및 드래그로 목표 발행: 맵 위에서 마우스를 이용해 새로운 target_pose를 발행합니다.
- 상태 흐름 시각화: 상태 머신의 현재 진행 상태를 다이어그램으로 표시합니다.
- 상태 및 오차 모니터링: 로봇의 현재 상태, 오차 등을 텍스트로 표시합니다.
- YAML 기반 동적 파라미터 제어: 'pid_config.yaml' 파일을 읽고 수정하여 모든 제어 파라미터를 동적으로 변경합니다.
- 다중 스레딩 및 QTimer: ROS 2 통신과 GUI 이벤트 루프가 충돌하지 않도록 하고, QTimer를 통해 주기적으로 GUI를 갱신하여 실시간 성을 보장합니다.
"""

import sys
import rclpy
import math
import yaml
import os
from rclpy.node import Node
from std_msgs.msg import String, Float64
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QDoubleSpinBox,
                             QGroupBox, QGridLayout, QPushButton, QCheckBox, QTextEdit)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer

# Matplotlib 임포트
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.patches import FancyBboxPatch

# ROS 노드: AMR의 현재 pose, goal_pose, state 구독 및 goal_pose 발행
class TurtleMonitor(Node):
    def __init__(self):
        super().__init__('turtle_monitor')
        self.amr_pose = None      # 현재 AMR의 pose (Odometry 메시지)
        self.goal_pose = None     # 현재 goal_pose
        self.current_state = None # 현재 상태 (문자열)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # 구독자 설정
        self.create_subscription(Odometry, '/odom_3', self.amr_pose_callback, 10)
        self.create_subscription(String, '/state', self.state_callback, 10)
        self.create_subscription(Float64, '/angle_error', self.angle_error_callback, 10)
        self.create_subscription(Float64, '/distance_error', self.distance_error_callback, 10)
        self.create_subscription(PoseStamped, '/camera_pose', self.camera_pose_callback, 10)
        self.create_subscription(PoseStamped, '/target_pose', self.target_pose_callback, 10)

        # 발행자 설정 - /goalpose3로 목표 발행 (도메인 브릿지를 통해 /goal_pose로 변환됨)
        self.target_publisher = self.create_publisher(PoseStamped, '/goal_pose3', 10)

        # 파라미터 서비스 클라이언트
        self.param_client = self.create_client(SetParameters, '/simple_robot_goal_controller/set_parameters')

        # 시그널들 (스레드 간 통신용)
        self.state_signal = None
        self.angle_error_signal = None
        self.distance_error_signal = None
        self.camera_pose_signal = None
        self.target_pose_signal = None

    def amr_pose_callback(self, msg):
        """Odometry 메시지에서 AMR 위치 추출"""
        self.amr_pose = msg
        
        # 위치 정보 추출
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # 쿼터니언에서 yaw 각도 추출
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def state_callback(self, msg):
        self.current_state = msg.data
        if self.state_signal:
            self.state_signal.emit(msg.data)

    def angle_error_callback(self, msg):
        if self.angle_error_signal:
            self.angle_error_signal.emit(msg.data)

    def distance_error_callback(self, msg):
        if self.distance_error_signal:
            self.distance_error_signal.emit(msg.data)

    def camera_pose_callback(self, msg):
        if self.camera_pose_signal:
            self.camera_pose_signal.emit(msg)

    def target_pose_callback(self, msg):
        self.goal_pose = msg
        if self.target_pose_signal:
            self.target_pose_signal.emit(msg)

    def publish_target_pose(self, msg):
        """목표 위치 발행"""
        self.target_publisher.publish(msg)
        self.get_logger().info(
            f"Published new goal: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}"
        )

    def set_parameter(self, name, value):
        """파라미터 설정"""
        if not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('파라미터 서비스에 연결할 수 없습니다.')
            return

        req = SetParameters.Request()
        if isinstance(value, bool):
            param = Parameter(name=name, 
                            value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value))
        else:
            param = Parameter(name=name, 
                            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value))
        req.parameters.append(param)
        self.param_client.call_async(req)

# ROS 2 노드 및 통신을 처리하는 스레드
class RosNodeThread(QThread):
    state_update = pyqtSignal(str)
    angle_error_update = pyqtSignal(float)
    distance_error_update = pyqtSignal(float)
    camera_pose_update = pyqtSignal(object)
    target_pose_update = pyqtSignal(object)

    def __init__(self):
        super().__init__()
        self.node = TurtleMonitor()
        
        # 노드의 시그널을 스레드 시그널에 연결
        self.node.state_signal = self.state_update
        self.node.angle_error_signal = self.angle_error_update
        self.node.distance_error_signal = self.distance_error_update
        self.node.camera_pose_signal = self.camera_pose_update
        self.node.target_pose_signal = self.target_pose_update

        self.current_state = "IDLE"

    def run(self):
        self.node.get_logger().info("ROS 2 모니터링 노드 스레드 시작")
        try:
            rclpy.spin(self.node)
        except Exception as e:
            self.node.get_logger().error(f"ROS 스레드 오류: {e}")
        finally:
            self.node.destroy_node()

    def publish_target_pose(self, msg):
        """목표 위치 발행"""
        self.node.publish_target_pose(msg)

    def set_parameter(self, name, value):
        """파라미터 설정"""
        self.node.set_parameter(name, value)

# 메인 GUI 윈도우
class MainWindow(QWidget):
    def __init__(self, ros_thread):
        super().__init__()
        self.ros_thread = ros_thread
        self.config_path = os.path.join(os.path.dirname(__file__), 'pid_config.yaml')
        
        # 상태 변수들
        self.camera_pose = None
        self.target_pose = None
        self.drag_start = None
        self.drag_current = None
        self.trajectory = []
        self.current_robot_state = "IDLE"

        # GUI 초기화
        self.init_ui()
        self.connect_signals()
        self.load_config()
        
        # 상태 리스트 (수정된 상태 이름들)
        self.state_list = ["IDLE", "RotateToGoal", "MoveToGoal", "RotateToFinal", "GoalReached"]
        
        # GUI 업데이트 타이머
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_gui_elements)
        self.timer.start(100)  # 100ms (10Hz) 업데이트

    def init_ui(self):
        """UI 초기화"""
        main_layout = QHBoxLayout()
        
        # 왼쪽: 맵 표시
        map_group = QGroupBox("AMR 맵")
        map_layout = QVBoxLayout()
        self.figure_map = Figure()
        self.canvas_map = FigureCanvas(self.figure_map)
        self.ax_map = self.figure_map.add_subplot(111)
        map_layout.addWidget(self.canvas_map)
        map_group.setLayout(map_layout)
        main_layout.addWidget(map_group, 2)  # 맵을 더 크게

        # 오른쪽 패널
        right_panel_layout = QVBoxLayout()
        
        # 상태 모니터링
        monitor_group = QGroupBox("상태 모니터링")
        monitor_layout = QGridLayout()
        self.state_label = QLabel("현재 상태: IDLE")
        self.angle_error_label = QLabel("각도 오차: 0.00 deg")
        self.distance_error_label = QLabel("거리 오차: 0.00 m")
        self.position_label = QLabel("현재 위치: (0.00, 0.00)")
        monitor_layout.addWidget(self.state_label, 0, 0, 1, 2)
        monitor_layout.addWidget(self.angle_error_label, 1, 0)
        monitor_layout.addWidget(self.distance_error_label, 1, 1)
        monitor_layout.addWidget(self.position_label, 2, 0, 1, 2)
        monitor_group.setLayout(monitor_layout)

        # 상태 흐름 다이어그램
        state_flow_group = QGroupBox("상태 흐름")
        state_flow_layout = QVBoxLayout()
        self.figure_state = Figure(figsize=(3, 4))
        self.canvas_state = FigureCanvas(self.figure_state)
        self.ax_state = self.figure_state.add_subplot(111)
        state_flow_layout.addWidget(self.canvas_state)
        state_flow_group.setLayout(state_flow_layout)

        # PID 파라미터 제어
        control_group = QGroupBox("PID 파라미터 제어")
        control_layout = QGridLayout()
        
        # 유클리드 거리 사용 옵션
        self.euclidean_checkbox = QCheckBox("유클리드 거리 사용")
        self.euclidean_checkbox.setChecked(True)
        control_layout.addWidget(self.euclidean_checkbox, 0, 0, 1, 2)

        # PID 파라미터 스핀박스들
        self.spinboxes = {}
        params = [
            (('tolerances', 'angle'), "각도 허용오차:"),
            (('tolerances', 'distance'), "거리 허용오차:"),
            (('angular', 'P'), "Angular P:"),
            (('angular', 'I'), "Angular I:"),
            (('angular', 'D'), "Angular D:"),
            (('angular', 'max_state'), "Angular Max:"),
            (('angular', 'min_state'), "Angular Min:"),
            (('linear', 'P'), "Linear P:"),
            (('linear', 'I'), "Linear I:"),
            (('linear', 'D'), "Linear D:"),
            (('linear', 'max_state'), "Linear Max:"),
            (('linear', 'min_state'), "Linear Min:")
        ]

        for i, ((group, param), label_text) in enumerate(params):
            label = QLabel(label_text)
            spinbox = QDoubleSpinBox()
            spinbox.setRange(-10.0, 10.0)
            spinbox.setDecimals(3)
            spinbox.setSingleStep(0.001)
            self.spinboxes[(group, param)] = spinbox
            control_layout.addWidget(label, i + 1, 0)
            control_layout.addWidget(spinbox, i + 1, 1)

        # 제어 버튼들
        self.save_button = QPushButton("YAML에 설정 저장")
        self.load_button = QPushButton("YAML에서 설정 로드")
        self.clear_trajectory_btn = QPushButton("궤적 지우기")
        control_layout.addWidget(self.save_button, len(params) + 1, 0)
        control_layout.addWidget(self.load_button, len(params) + 1, 1)
        control_layout.addWidget(self.clear_trajectory_btn, len(params) + 2, 0, 1, 2)
        control_group.setLayout(control_layout)

        # 로그 텍스트 (추가)
        log_group = QGroupBox("시스템 로그")
        log_layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setMaximumHeight(100)
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)

        # 오른쪽 패널 조립
        right_panel_layout.addWidget(monitor_group)
        right_panel_layout.addWidget(state_flow_group)
        right_panel_layout.addWidget(control_group)
        right_panel_layout.addWidget(log_group)
        
        main_layout.addLayout(right_panel_layout, 1)

        self.setLayout(main_layout)
        self.setWindowTitle('AMR PID Control GUI')
        self.setGeometry(100, 100, 1400, 900)

    def connect_signals(self):
        """시그널 연결"""
        # ROS 시그널 연결
        self.ros_thread.state_update.connect(self.update_state_label)
        self.ros_thread.angle_error_update.connect(
            lambda e: self.angle_error_label.setText(f"각도 오차: {math.degrees(e):.2f} deg"))
        self.ros_thread.distance_error_update.connect(
            lambda e: self.distance_error_label.setText(f"거리 오차: {e:.3f} m"))
        self.ros_thread.camera_pose_update.connect(self.handle_camera_pose_update)
        self.ros_thread.target_pose_update.connect(self.handle_target_pose_update)

        # 마우스 이벤트 연결
        self.canvas_map.mpl_connect('button_press_event', self.on_mouse_press)
        self.canvas_map.mpl_connect('motion_notify_event', self.on_mouse_move)
        self.canvas_map.mpl_connect('button_release_event', self.on_mouse_release)

        # UI 버튼 연결
        self.euclidean_checkbox.stateChanged.connect(self.update_distance_mode)
        self.save_button.clicked.connect(self.save_config)
        self.load_button.clicked.connect(self.load_config)
        self.clear_trajectory_btn.clicked.connect(self.clear_trajectory)

    def update_state_label(self, state):
        """상태 라벨 업데이트"""
        self.state_label.setText(f"현재 상태: {state}")
        self.current_robot_state = state
        
        # 현재 위치 업데이트
        if self.ros_thread.node.amr_pose:
            x = self.ros_thread.node.current_x
            y = self.ros_thread.node.current_y
            self.position_label.setText(f"현재 위치: ({x:.3f}, {y:.3f})")

    def load_config(self):
        """YAML 설정 파일에서 파라미터 로드"""
        try:
            if os.path.exists(self.config_path):
                with open(self.config_path, 'r') as f:
                    config = yaml.safe_load(f)
                
                # 스핀박스에 값 설정
                for (group, param), spinbox in self.spinboxes.items():
                    if group in config and param in config[group]:
                        spinbox.setValue(float(config[group][param]))
                
                self.add_log("YAML 설정을 GUI에 로드했습니다.")
            else:
                self.add_log("설정 파일이 없습니다. 기본값을 사용합니다.")
                self.create_default_config()
        except Exception as e:
            self.add_log(f"YAML 설정 파일 로드 실패: {e}")

    def save_config(self):
        """현재 GUI 설정을 YAML 파일에 저장"""
        try:
            # 기존 설정 로드 (있으면)
            config = {}
            if os.path.exists(self.config_path):
                with open(self.config_path, 'r') as f:
                    config = yaml.safe_load(f) or {}

            # GUI에서 설정 업데이트
            for (group, param), spinbox in self.spinboxes.items():
                if group not in config:
                    config[group] = {}
                config[group][param] = spinbox.value()
            
            # 유클리드 거리 옵션 저장
            config['use_euclidean_distance'] = self.euclidean_checkbox.isChecked()
            
            # 파일에 저장
            with open(self.config_path, 'w') as f:
                yaml.dump(config, f, default_flow_style=False, sort_keys=False)
            
            self.add_log(f"설정을 '{self.config_path}'에 저장했습니다.")
        except Exception as e:
            self.add_log(f"YAML 설정 파일 저장 실패: {e}")

    def create_default_config(self):
        """기본 설정 생성 및 적용"""
        default_values = {
            ('tolerances', 'angle'): 0.1,
            ('tolerances', 'distance'): 0.05,
            ('angular', 'P'): 1.0,
            ('angular', 'I'): 0.0,
            ('angular', 'D'): 0.1,
            ('angular', 'max_state'): 2.0,
            ('angular', 'min_state'): -2.0,
            ('linear', 'P'): 0.5,
            ('linear', 'I'): 0.0,
            ('linear', 'D'): 0.1,
            ('linear', 'max_state'): 1.0,
            ('linear', 'min_state'): -1.0
        }
        
        for (group, param), value in default_values.items():
            if (group, param) in self.spinboxes:
                self.spinboxes[(group, param)].setValue(value)

    def update_distance_mode(self, state):
        """거리 계산 모드 업데이트"""
        use_euclidean = (state == Qt.Checked)
        self.ros_thread.set_parameter('use_euclidean_distance', use_euclidean)
        self.add_log(f"유클리드 거리 사용: {use_euclidean}")

    def clear_trajectory(self):
        """궤적 지우기"""
        self.trajectory.clear()
        self.add_log("궤적을 지웠습니다.")

    def handle_camera_pose_update(self, msg):
        """카메라 포즈 업데이트 처리"""
        self.camera_pose = msg
        if self.camera_pose:
            x, y = msg.pose.position.x, msg.pose.position.y
            self.trajectory.append((x, y))
            # 궤적 길이 제한 (메모리 관리)
            if len(self.trajectory) > 500:
                self.trajectory.pop(0)

    def handle_target_pose_update(self, msg):
        """타겟 포즈 업데이트 처리"""
        self.target_pose = msg

    def on_mouse_press(self, event):
        """마우스 누르기 이벤트"""
        if event.button == 1 and event.inaxes == self.ax_map:  # 왼쪽 버튼
            self.drag_start = (event.xdata, event.ydata)
            self.drag_current = (event.xdata, event.ydata)

    def on_mouse_move(self, event):
        """마우스 이동 이벤트"""
        if self.drag_start and event.inaxes == self.ax_map:
            self.drag_current = (event.xdata, event.ydata)

    def on_mouse_release(self, event):
        """마우스 놓기 이벤트"""
        if event.button == 1 and self.drag_start and self.drag_current:
            # 드래그 방향으로 목표 자세 계산
            dx = self.drag_current[0] - self.drag_start[0]
            dy = self.drag_current[1] - self.drag_start[1]
            
            # 최소 드래그 거리 확인
            if math.hypot(dx, dy) < 0.02:  # 2cm 미만이면 단순 클릭으로 간주
                theta = 0.0  # 기본 자세
            else:
                theta = math.atan2(dy, dx)

            # 목표 메시지 생성
            target_msg = PoseStamped()
            target_msg.header.stamp = self.ros_thread.node.get_clock().now().to_msg()
            target_msg.header.frame_id = 'map'
            target_msg.pose.position.x = float(self.drag_start[0])
            target_msg.pose.position.y = float(self.drag_start[1])
            target_msg.pose.position.z = 0.0
            
            # 쿼터니언으로 자세 설정
            target_msg.pose.orientation.z = math.sin(theta / 2.0)
            target_msg.pose.orientation.w = math.cos(theta / 2.0)

            # 목표 발행
            self.ros_thread.publish_target_pose(target_msg)
            
            self.add_log(
                f"새 목표: ({target_msg.pose.position.x:.2f}, {target_msg.pose.position.y:.2f}), "
                f"각도: {math.degrees(theta):.1f}°"
            )

        self.drag_start = None
        self.drag_current = None

    def update_gui_elements(self):
        """GUI 요소들 주기적 업데이트"""
        self.update_map_display()
        self.update_state_diagram()

    def update_map_display(self):
        """맵 디스플레이 업데이트"""
        self.ax_map.clear()
        self.ax_map.set_xlim(-1.0, 3.0)  # 적절한 맵 범위 설정
        self.ax_map.set_ylim(-1.0, 2.0)
        self.ax_map.set_aspect('equal')
        self.ax_map.set_title("AMR Map (Click to Set Target)")
        self.ax_map.set_xlabel("X (m)")
        self.ax_map.set_ylabel("Y (m)")
        self.ax_map.grid(True, linestyle='--', linewidth=0.5, alpha=0.7)

        # 궤적 표시
        if len(self.trajectory) > 1:
            traj_x, traj_y = zip(*self.trajectory)
            self.ax_map.plot(traj_x, traj_y, 'g-', linewidth=2, alpha=0.7, label='궤적')

        # 현재 AMR 위치 표시 (Odometry에서)
        if self.ros_thread.node.amr_pose:
            x = self.ros_thread.node.current_x
            y = self.ros_thread.node.current_y
            yaw = self.ros_thread.node.current_yaw
            
            # AMR 위치와 방향
            self.ax_map.plot(x, y, 'bo', markersize=10, label='AMR')
            arrow_length = 0.15
            self.ax_map.arrow(x, y, arrow_length * math.cos(yaw), arrow_length * math.sin(yaw), 
                             head_width=0.05, head_length=0.03, fc='blue', ec='blue')

        # 목표 위치 표시
        if self.target_pose:
            x, y = self.target_pose.pose.position.x, self.target_pose.pose.position.y
            qz, qw = self.target_pose.pose.orientation.z, self.target_pose.pose.orientation.w
            yaw = 2 * math.atan2(qz, qw)
            
            # 목표 위치와 방향
            self.ax_map.plot(x, y, 'rX', markersize=12, markeredgewidth=3, label='목표')
            arrow_length = 0.15
            self.ax_map.arrow(x, y, arrow_length * math.cos(yaw), arrow_length * math.sin(yaw), 
                             head_width=0.05, head_length=0.03, fc='red', ec='red')

        # 드래그 중인 화살표 표시
        if self.drag_start and self.drag_current:
            sx, sy = self.drag_start
            cx, cy = self.drag_current
            self.ax_map.arrow(sx, sy, cx - sx, cy - sy, 
                             head_width=0.04, head_length=0.03, fc='orange', ec='orange', 
                             linestyle='--', alpha=0.8)
            
        self.ax_map.legend(loc='upper right')
        self.canvas_map.draw()

    def update_state_diagram(self):
        """상태 다이어그램 업데이트"""
        self.ax_state.clear()
        self.ax_state.set_xlim(0, 1.5)
        self.ax_state.set_ylim(0, 1)
        self.ax_state.axis('off')
        
        # 상태 박스 설정
        block_width, block_height, spacing = 1.2, 0.15, 0.05
        start_x = (1.5 - block_width) / 2
        n = len(self.state_list)
        total_height = n * block_height + (n - 1) * spacing
        group_bottom = 0.5 - total_height / 2
        
        for i, state in enumerate(self.state_list):
            y = group_bottom + (n - 1 - i) * (block_height + spacing)
            
            # 현재 상태에 따라 색상 변경
            if self.current_robot_state == state:
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
        
        self.canvas_state.draw()

    def add_log(self, message):
        """로그 메시지 추가"""
        self.log_text.append(f"[{self.get_current_time()}] {message}")
        # 자동으로 맨 아래로 스크롤
        self.log_text.verticalScrollBar().setValue(
            self.log_text.verticalScrollBar().maximum())

    def get_current_time(self):
        """현재 시간 문자열 반환"""
        from datetime import datetime
        return datetime.now().strftime("%H:%M:%S")

def main(args=None):
    """메인 함수"""
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    
    # ROS 스레드 시작
    ros_thread = RosNodeThread()
    
    # 메인 윈도우 생성 및 표시
    main_win = MainWindow(ros_thread)
    main_win.show()
    
    # ROS 스레드 시작
    ros_thread.start()
    
    # GUI 종료 처리
    try:
        exit_code = app.exec_()
    finally:
        ros_thread.node.get_logger().info("GUI 종료. ROS 2 노드 종료 중...")
        rclpy.shutdown()
    
    sys.exit(exit_code)

if __name__ == '__main__':
    main()