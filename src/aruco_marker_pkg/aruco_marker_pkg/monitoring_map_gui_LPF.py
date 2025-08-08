#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import math
import threading
import sys
import numpy as np

from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout
from PyQt5.QtCore import QTimer
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

NUM_ROBOTS = 3  # 필요에 따라 로봇 수 조정

# ✅ 필터링 파라미터
ALPHA = 0.3  # Low Pass Filter 계수 (0~1, 낮을수록 더 부드럽게)
MIN_DISTANCE_THRESHOLD = 0.01  # 최소 움직임 임계값 (m)
MIN_ANGLE_THRESHOLD = 0.05  # 최소 각도 변화 임계값 (rad, 약 2.9도)

# ✅ 직접 정의한 맵 (1 = 벽, 0 = 자유공간)
custom_map = [
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,1,0,0,0,0,1,1,1,1,1,0,0,0,1],
    [1,0,0,0,0,0,0,0,1,0,0,0,0,1,1,1,1,1,0,0,0,1],
    [1,0,0,0,0,0,0,0,1,0,0,0,0,1,1,1,1,1,0,0,0,1],
    [1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
]


class RobotMonitor(Node):
    def __init__(self):
        super().__init__('multi_robot_monitor')
        self.robot_poses = {}  # {id: (x, y, theta)} - 원본 위치 (ROS 통신용)
        self.robot_goals = {}  # {id: (x, y)}
        self.robot_paths = {}  # {id: [(x1, y1), (x2, y2), ...]}
        
        # ✅ GUI 표시용 필터링된 위치 데이터
        self.filtered_poses = {}  # {id: (x, y, theta)} - 필터링된 위치 (GUI 표시용)

        for i in range(1, NUM_ROBOTS + 1):
            # 기존 구독자들
            self.create_subscription(Odometry, f'/odom_{i}', self.make_odom_cb(i), 10)
            self.create_subscription(PoseStamped, f'/goalpose{i}', self.make_goal_cb(i), 10)
            
            # ✅ 경로 구독자 추가
            self.create_subscription(Path, f'/path{i}', self.make_path_cb(i), 10)

    def make_odom_cb(self, robot_id):
        def callback(msg):
            pose = msg.pose.pose
            # 원본 위치 저장 (ROS 통신용)
            raw_x = pose.position.x
            raw_y = pose.position.y
            raw_theta = self.yaw_from_quat(pose.orientation)
            
            self.robot_poses[robot_id] = (raw_x, raw_y, raw_theta)
            
            # ✅ GUI용 필터링된 위치 계산
            self.update_filtered_pose(robot_id, raw_x, raw_y, raw_theta)
            
        return callback

    def update_filtered_pose(self, robot_id, raw_x, raw_y, raw_theta):
        """GUI 표시용 위치에 Low Pass Filter와 최소 변화량 임계값 적용"""
        
        if robot_id not in self.filtered_poses:
            # 첫 번째 데이터는 그대로 사용
            self.filtered_poses[robot_id] = (raw_x, raw_y, raw_theta)
            return
        
        prev_x, prev_y, prev_theta = self.filtered_poses[robot_id]
        
        # ✅ 1. 최소 변화량 체크
        distance_change = math.sqrt((raw_x - prev_x)**2 + (raw_y - prev_y)**2)
        angle_change = abs(self.normalize_angle(raw_theta - prev_theta))
        
        # 변화량이 임계값보다 작으면 이전 값 유지
        if distance_change < MIN_DISTANCE_THRESHOLD and angle_change < MIN_ANGLE_THRESHOLD:
            return  # 필터링된 위치 업데이트 안함
        
        # ✅ 2. Low Pass Filter 적용
        filtered_x = ALPHA * raw_x + (1 - ALPHA) * prev_x
        filtered_y = ALPHA * raw_y + (1 - ALPHA) * prev_y
        
        # 각도는 순환 특성을 고려한 필터링
        angle_diff = self.normalize_angle(raw_theta - prev_theta)
        filtered_theta = prev_theta + ALPHA * angle_diff
        filtered_theta = self.normalize_angle(filtered_theta)
        
        # 필터링된 위치 업데이트
        self.filtered_poses[robot_id] = (filtered_x, filtered_y, filtered_theta)

    def normalize_angle(self, angle):
        """각도를 -π ~ π 범위로 정규화"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def make_goal_cb(self, robot_id):
        def callback(msg):
            pose = msg.pose
            self.robot_goals[robot_id] = (pose.position.x, pose.position.y)
        return callback

    def make_path_cb(self, robot_id):
        """✅ 경로 콜백 함수"""
        def callback(msg):
            path_points = []
            for pose in msg.poses:
                x = pose.pose.position.x
                y = pose.pose.position.y
                path_points.append((x, y))
            self.robot_paths[robot_id] = path_points
            self.get_logger().info(f'🛤️  로봇 {robot_id} 경로 수신: {len(path_points)} 점')
        return callback

    def yaw_from_quat(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

def ros_spin(node):
    rclpy.spin(node)

class MainWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.setWindowTitle("Multi-Robot Monitor with Stabilized Display")
        self.node = node

        # ✅ 창 크기 1.5배 확대
        self.resize(1500, 750)

        # ✅ 맵을 numpy로 변환
        self.map_array = np.array(custom_map, dtype=np.uint8)
        self.map_array = (1 - self.map_array) * 255  # 0 → 255 (white), 1 → 0 (black)
        self.map_array = np.flipud(self.map_array)   # y축 상하반전

        self.map_resolution = 2.0 / 22  # 2.0m / 22 cells → 0.0909m/cell (임의로 지정)

        # UI
        main_widget = QWidget()
        layout = QVBoxLayout()
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)
        main_widget.setLayout(layout)
        self.setCentralWidget(main_widget)

        self.ax = self.figure.add_subplot(111)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(100)

    def update_plot(self):
        self.ax.clear()

        # ✅ 맵 출력: 범위 (-0.1, -0.1) ~ (2.1, 1.1)
        extent = [-0.1, 2.1, -0.1, 1.1]
        self.ax.imshow(self.map_array, cmap='gray', origin='lower', extent=extent)

        # ✅ 각 로봇의 경로를 30% 투명도로 표시
        for rid in range(1, NUM_ROBOTS + 1):
            if rid in self.node.robot_paths and len(self.node.robot_paths[rid]) > 1:
                path_points = self.node.robot_paths[rid]
                x_coords = [point[0] for point in path_points]
                y_coords = [point[1] for point in path_points]
                
                # 경로를 선으로 연결하여 표시 (30% 투명도)
                self.ax.plot(x_coords, y_coords, '-', 
                           color=f'C{rid}', linewidth=2, alpha=0.3, 
                           label=f'Robot {rid} Path')
                
                # 경로 시작점과 끝점 표시
                if len(path_points) > 0:
                    # 시작점 (원)
                    self.ax.plot(x_coords[0], y_coords[0], 'o', 
                               color=f'C{rid}', markersize=8, alpha=0.7)
                    # 끝점 (사각형)
                    self.ax.plot(x_coords[-1], y_coords[-1], 's', 
                               color=f'C{rid}', markersize=8, alpha=0.7)

        # ✅ 로봇 현재 위치 표시 (필터링된 위치 사용)
        for rid in range(1, NUM_ROBOTS + 1):
            # 필터링된 위치 사용 (GUI 표시용)
            if rid in self.node.filtered_poses:
                x, y, theta = self.node.filtered_poses[rid]
                dx = 0.05 * math.cos(theta)
                dy = 0.05 * math.sin(theta)
                # 로봇 현재 위치 (화살표)
                self.ax.arrow(x, y, dx, dy, head_width=0.02, 
                            color=f'C{rid}', alpha=1.0, linewidth=2)
                # 로봇 현재 위치 (점)
                self.ax.plot(x, y, 'o', color=f'C{rid}', markersize=10, 
                           markeredgecolor='black', markeredgewidth=1)
                
                # 디버그 정보 표시 (선택사항)
                if rid in self.node.robot_poses:
                    raw_x, raw_y, _ = self.node.robot_poses[rid]
                    distance = math.sqrt((x - raw_x)**2 + (y - raw_y)**2)
                    if distance > 0.001:  # 차이가 있을 때만 표시
                        # 원본 위치를 작은 점으로 표시 (디버그용)
                        self.ax.plot(raw_x, raw_y, '.', color=f'C{rid}', 
                                   markersize=4, alpha=0.5)

            # 목표 위치 표시
            if rid in self.node.robot_goals:
                gx, gy = self.node.robot_goals[rid]
                # 현재 목표점 (X 표시)
                self.ax.plot(gx, gy, 'X', color=f'C{rid}', markersize=8, 
                           markeredgecolor='black', markeredgewidth=1)

        # ✅ 눈금, 범위 설정
        self.ax.set_xlim(-0.1, 2.1)
        self.ax.set_ylim(-0.1, 1.1)
        self.ax.set_xticks(np.arange(-0.1, 2.11, 0.1))
        self.ax.set_yticks(np.arange(-0.1, 1.11, 0.1))

        # 필터 정보를 제목에 표시
        title = f"Multi-Robot Monitor (LPF α={ALPHA}, Min Δ={MIN_DISTANCE_THRESHOLD}m)"
        self.ax.set_title(title)
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.grid(True, alpha=0.3)
        
        # 범례 추가 (경로가 있는 로봇만)
        handles, labels = self.ax.get_legend_handles_labels()
        if handles:
            self.ax.legend(loc='upper right', fontsize=8)
        
        self.canvas.draw()

def main(args=None):
    rclpy.init(args=args)
    node = RobotMonitor()

    spin_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    window = MainWindow(node)
    window.show()
    ret = app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)

if __name__ == '__main__':
    main()