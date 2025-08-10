#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64
import math
import numpy as np
import time

def normalize_angle(angle):
    """Normalize angle to [-pi, pi]"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

class DWBTrajectory:
    """DWB 궤적 클래스"""
    def __init__(self, v, w, dt=0.1, sim_time=1.0):
        self.v = v  # 선속도
        self.w = w  # 각속도
        self.dt = dt
        self.sim_time = sim_time
        self.poses = []
        self.cost = float('inf')
        
    def simulate(self, start_pose, start_yaw):
        """궤적 시뮬레이션"""
        self.poses = []
        x, y, theta = start_pose[0], start_pose[1], start_yaw
        
        for i in range(int(self.sim_time / self.dt)):
            # 운동학적 모델로 위치 업데이트
            x += self.v * math.cos(theta) * self.dt
            y += self.v * math.sin(theta) * self.dt
            theta += self.w * self.dt
            theta = normalize_angle(theta)
            
            self.poses.append([x, y, theta])
            
        return self.poses

class DWBLocalPlanner(Node):
    def __init__(self):
        super().__init__('dwb_local_planner')
        
        # 상태 변수
        self.current_pose = None
        self.yaw = 0.0
        self.waypoints = []
        self.current_waypoint_idx = 0
        
        # DWB 파라미터 선언
        self.declare_parameter('max_vel_x', 2.0)
        self.declare_parameter('min_vel_x', 0.0)
        self.declare_parameter('max_vel_theta', 3.0)
        self.declare_parameter('min_vel_theta', -3.0)
        
        self.declare_parameter('acc_lim_x', 2.0)
        self.declare_parameter('acc_lim_theta', 3.0)
        
        self.declare_parameter('vel_samples', 10)
        self.declare_parameter('theta_samples', 20)
        
        self.declare_parameter('sim_time', 1.0)
        self.declare_parameter('sim_granularity', 0.1)
        
        # 비용 함수 가중치
        self.declare_parameter('path_distance_bias', 10.0)
        self.declare_parameter('goal_distance_bias', 5.0)
        self.declare_parameter('occdist_scale', 0.1)
        self.declare_parameter('speed_bias', 1.0)
        
        # 허용 오차
        self.declare_parameter('xy_goal_tolerance', 0.05)
        self.declare_parameter('yaw_goal_tolerance', 0.1)
        
        # 파라미터 업데이트
        self.update_parameters()
        
        # 이전 속도 (가속도 제한용)
        self.last_vel_x = 0.0
        self.last_vel_theta = 0.0
        self.last_time = time.time()
        
        # ROS 구독자와 퍼블리셔
        self.odom_sub = self.create_subscription(Odometry, '/odom_1', self.odom_callback, 10)
        self.path_sub = self.create_subscription(Path, '/waypoint_path', self.path_callback, 10)
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.current_goal_pub = self.create_publisher(PoseStamped, '/current_goal', 10)
        
        # 성능 모니터링
        self.distance_error_pub = self.create_publisher(Float64, '/distance_error', 10)
        self.speed_pub = self.create_publisher(Float64, '/current_speed', 10)
        
        # 제어 루프 타이머
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz
        
        self.get_logger().info("DWB Local Planner 초기화 완료")

    def update_parameters(self):
        """파라미터 값들을 업데이트"""
        self.max_vel_x = self.get_parameter('max_vel_x').value
        self.min_vel_x = self.get_parameter('min_vel_x').value
        self.max_vel_theta = self.get_parameter('max_vel_theta').value
        self.min_vel_theta = self.get_parameter('min_vel_theta').value
        
        self.acc_lim_x = self.get_parameter('acc_lim_x').value
        self.acc_lim_theta = self.get_parameter('acc_lim_theta').value
        
        self.vel_samples = int(self.get_parameter('vel_samples').value)
        self.theta_samples = int(self.get_parameter('theta_samples').value)
        
        self.sim_time = self.get_parameter('sim_time').value
        self.sim_granularity = self.get_parameter('sim_granularity').value
        
        # 비용 함수 가중치
        self.path_distance_bias = self.get_parameter('path_distance_bias').value
        self.goal_distance_bias = self.get_parameter('goal_distance_bias').value
        self.occdist_scale = self.get_parameter('occdist_scale').value
        self.speed_bias = self.get_parameter('speed_bias').value
        
        # 허용 오차
        self.xy_goal_tolerance = self.get_parameter('xy_goal_tolerance').value
        self.yaw_goal_tolerance = self.get_parameter('yaw_goal_tolerance').value

    def odom_callback(self, msg):
        """오도메트리 콜백"""
        self.current_pose = msg.pose.pose
        
        # 쿼터니언에서 yaw 각도 추출
        q = self.current_pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def path_callback(self, msg):
        """웨이포인트 경로 콜백"""
        self.waypoints = []
        for pose_stamped in msg.poses:
            point = [pose_stamped.pose.position.x, pose_stamped.pose.position.y]
            self.waypoints.append(point)
        
        self.current_waypoint_idx = 0
        self.get_logger().info(f"새로운 경로 수신: {len(self.waypoints)}개 웨이포인트")

    def get_current_goal(self):
        """현재 목표점 반환"""
        if not self.waypoints or self.current_waypoint_idx >= len(self.waypoints):
            return None
        return self.waypoints[self.current_waypoint_idx]

    def generate_velocity_samples(self):
        """속도 샘플 생성 (가속도 제한 고려)"""
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0:
            dt = 0.05
        
        # 가속도 제한을 고려한 속도 범위 계산
        max_vel_x_acc = self.last_vel_x + self.acc_lim_x * dt
        min_vel_x_acc = self.last_vel_x - self.acc_lim_x * dt
        
        max_vel_theta_acc = self.last_vel_theta + self.acc_lim_theta * dt
        min_vel_theta_acc = self.last_vel_theta - self.acc_lim_theta * dt
        
        # 실제 속도 범위
        vel_x_max = min(self.max_vel_x, max_vel_x_acc)
        vel_x_min = max(self.min_vel_x, min_vel_x_acc)
        
        vel_theta_max = min(self.max_vel_theta, max_vel_theta_acc)
        vel_theta_min = max(self.min_vel_theta, min_vel_theta_acc)
        
        # 속도 샘플 생성
        velocity_samples = []
        
        for i in range(self.vel_samples):
            for j in range(self.theta_samples):
                v = vel_x_min + (vel_x_max - vel_x_min) * i / max(1, self.vel_samples - 1)
                w = vel_theta_min + (vel_theta_max - vel_theta_min) * j / max(1, self.theta_samples - 1)
                velocity_samples.append((v, w))
        
        return velocity_samples

    def calculate_path_distance_cost(self, trajectory):
        """경로 추종 비용 계산"""
        if not self.waypoints:
            return 0.0
        
        min_distance = float('inf')
        
        # 궤적의 각 점에서 웨이포인트까지의 최소 거리 찾기
        for pose in trajectory.poses:
            for waypoint in self.waypoints[self.current_waypoint_idx:]:
                dist = math.hypot(pose[0] - waypoint[0], pose[1] - waypoint[1])
                min_distance = min(min_distance, dist)
        
        return min_distance

    def calculate_goal_distance_cost(self, trajectory):
        """목표점 거리 비용 계산"""
        goal = self.get_current_goal()
        if goal is None or not trajectory.poses:
            return 0.0
        
        # 궤적 끝점에서 목표점까지의 거리
        final_pose = trajectory.poses[-1]
        return math.hypot(final_pose[0] - goal[0], final_pose[1] - goal[1])

    def calculate_speed_cost(self, trajectory):
        """속도 비용 계산 (빠른 속도에 더 높은 보상)"""
        return -abs(trajectory.v)  # 음수로 하여 빠른 속도가 낮은 비용

    def calculate_trajectory_cost(self, trajectory):
        """전체 궤적 비용 계산"""
        path_cost = self.calculate_path_distance_cost(trajectory)
        goal_cost = self.calculate_goal_distance_cost(trajectory)
        speed_cost = self.calculate_speed_cost(trajectory)
        
        total_cost = (self.path_distance_bias * path_cost +
                     self.goal_distance_bias * goal_cost +
                     self.speed_bias * speed_cost)
        
        return total_cost

    def select_best_trajectory(self, trajectories):
        """최적 궤적 선택"""
        if not trajectories:
            return None
        
        best_trajectory = None
        best_cost = float('inf')
        
        for trajectory in trajectories:
            cost = self.calculate_trajectory_cost(trajectory)
            trajectory.cost = cost
            
            if cost < best_cost:
                best_cost = cost
                best_trajectory = trajectory
        
        return best_trajectory

    def update_waypoint_progress(self):
        """웨이포인트 진행 상황 업데이트"""
        if not self.waypoints or self.current_pose is None:
            return
        
        current_goal = self.get_current_goal()
        if current_goal is None:
            return
        
        # 현재 목표점까지의 거리 계산
        dx = current_goal[0] - self.current_pose.position.x
        dy = current_goal[1] - self.current_pose.position.y
        distance = math.hypot(dx, dy)
        
        # 목표점 도달 확인
        if distance < self.xy_goal_tolerance:
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx < len(self.waypoints):
                self.get_logger().info(f"웨이포인트 {self.current_waypoint_idx-1} 도달. 다음 목표로 이동.")
            else:
                self.get_logger().info("✅ 모든 웨이포인트 도달 완료!")

    def control_loop(self):
        """메인 제어 루프"""
        if self.current_pose is None or not self.waypoints:
            return
        
        # 웨이포인트 진행 상황 업데이트
        self.update_waypoint_progress()
        
        current_goal = self.get_current_goal()
        if current_goal is None:
            # 모든 웨이포인트 완료
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            return
        
        # 현재 목표점 발행 (시각화용)
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = current_goal[0]
        goal_msg.pose.position.y = current_goal[1]
        self.current_goal_pub.publish(goal_msg)
        
        # 속도 샘플 생성
        velocity_samples = self.generate_velocity_samples()
        
        # 궤적 생성 및 평가
        trajectories = []
        start_pose = [self.current_pose.position.x, self.current_pose.position.y]
        
        for v, w in velocity_samples:
            trajectory = DWBTrajectory(v, w, self.sim_granularity, self.sim_time)
            trajectory.simulate(start_pose, self.yaw)
            trajectories.append(trajectory)
        
        # 최적 궤적 선택
        best_trajectory = self.select_best_trajectory(trajectories)
        
        if best_trajectory is None:
            self.get_logger().warn("최적 궤적을 찾을 수 없습니다.")
            cmd = Twist()
        else:
            cmd = Twist()
            cmd.linear.x = best_trajectory.v
            cmd.angular.z = best_trajectory.w
            
            # 이전 속도 업데이트
            self.last_vel_x = best_trajectory.v
            self.last_vel_theta = best_trajectory.w
            self.last_time = time.time()
        
        # 속도 명령 발행
        self.cmd_pub.publish(cmd)
        
        # 모니터링 정보 발행
        if current_goal:
            dx = current_goal[0] - self.current_pose.position.x
            dy = current_goal[1] - self.current_pose.position.y
            distance = math.hypot(dx, dy)
            
            distance_msg = Float64()
            distance_msg.data = distance
            self.distance_error_pub.publish(distance_msg)
        
        speed_msg = Float64()
        speed_msg.data = math.hypot(cmd.linear.x, cmd.angular.z)
        self.speed_pub.publish(speed_msg)
        
        # 주기적 로깅
        if self.get_clock().now().nanoseconds % 1000000000 < 50000000:  # 1초마다
            self.get_logger().info(
                f"목표 {self.current_waypoint_idx}/{len(self.waypoints)}: "
                f"({current_goal[0]:.2f}, {current_goal[1]:.2f}) | "
                f"거리: {distance:.3f}m | "
                f"cmd: v={cmd.linear.x:.2f}, w={cmd.angular.z:.2f}"
            )

def main(args=None):
    rclpy.init(args=args)
    
    node = DWBLocalPlanner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("사용자에 의해 중단됨")
    except Exception as e:
        node.get_logger().error(f"오류 발생: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()