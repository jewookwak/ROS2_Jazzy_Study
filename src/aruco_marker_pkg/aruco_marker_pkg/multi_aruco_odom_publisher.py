# # #!/usr/bin/env python3
# # import cv2
# # import numpy as np
# # import rclpy
# # from rclpy.node import Node
# # from geometry_msgs.msg import PoseStamped
# # from nav_msgs.msg import Odometry
# # import math
# # import time

# # from tf2_ros import TransformBroadcaster
# # from geometry_msgs.msg import TransformStamped


# # def yaw_to_quaternion(yaw):
# #     """Z축 회전(yaw, radian)을 쿼터니언(x, y, z, w)으로 변환."""
# #     q = {}
# #     q['x'] = 0.0
# #     q['y'] = 0.0
# #     q['z'] = math.sin(yaw / 2.0)
# #     q['w'] = math.cos(yaw / 2.0)
# #     return q


# # def normalize_angle(angle):
# #     """각도를 -π ~ π 범위로 정규화"""
# #     while angle > math.pi:
# #         angle -= 2.0 * math.pi
# #     while angle < -math.pi:
# #         angle += 2.0 * math.pi
# #     return angle


# # class ArucoDetector:
# #     def __init__(self, camera_matrix, dist_coeffs, marker_length=0.05):
# #         self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
# #         self.aruco_params = cv2.aruco.DetectorParameters_create()
# #         self.camera_matrix = camera_matrix
# #         self.dist_coeffs = dist_coeffs
# #         self.marker_length = marker_length

# #         self.src_pts = np.array([
# #             [-1.564,  0.090],  
# #             [-1.637, -0.847],  
# #             [ 0.382,  0.057],  
# #             [ 0.342, -0.854]
# #         ], dtype=np.float32)
# #         self.dst_pts = np.array([
# #             [0.05, 0.05],
# #             [0.05, 0.95],
# #             [1.95, 0.05],
# #             [1.95, 0.95]
# #         ], dtype=np.float32)
# #         self.H = cv2.getPerspectiveTransform(self.src_pts, self.dst_pts)

# #     def transform_point(self, pt):
# #         src = np.array([pt[0], pt[1], 1.0], dtype=np.float32)
# #         dst = self.H @ src
# #         dst = dst / dst[2]
# #         return float(dst[0]), float(dst[1])

# #     def detect_markers(self, frame):
# #         corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)
# #         detected = []
# #         if ids is not None:
# #             rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
# #                 corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
# #             for i in range(len(ids)):
# #                 marker_id = int(ids[i][0])
# #                 tvec = tvecs[i][0]
# #                 rvec = rvecs[i][0]
# #                 x, y = self.transform_point((tvec[0], tvec[1]))
# #                 x, y = round(x, 2), round(y, 2)

# #                 rot_mat, _ = cv2.Rodrigues(rvec)
# #                 yaw = -(np.arctan2(rot_mat[1, 0], rot_mat[0, 0]) - np.pi / 2)
# #                 yaw = normalize_angle(yaw)
# #                 yaw = round(float(yaw), 3)

# #                 corners_reshaped = corners[i].reshape((4, 2))
# #                 center_x = int(np.mean(corners_reshaped[:, 0]))
# #                 center_y = int(np.mean(corners_reshaped[:, 1]))

# #                 detected.append({
# #                     "id": marker_id,
# #                     "robot_xy": [x, y],
# #                     "yaw": yaw,
# #                     "text_pos": (center_x, center_y)
# #                 })
        
# #         return detected


# # class RobotState:
# #     """각 로봇의 상태를 저장하는 클래스"""
# #     def __init__(self):
# #         self.x = 0.0
# #         self.y = 0.0
# #         self.yaw = 0.0
# #         self.vx = 0.0
# #         self.vy = 0.0
# #         self.vyaw = 0.0
# #         self.last_time = None
# #         self.last_x = 0.0
# #         self.last_y = 0.0
# #         self.last_yaw = 0.0
# #         self.initialized = False


# # class ArucoOdometryPublisher(Node):
# #     def __init__(self, camera_matrix, dist_coeffs):
# #         super().__init__('aruco_odometry_publisher')
# #         self.detector = ArucoDetector(camera_matrix, dist_coeffs)
# #         self.cap = cv2.VideoCapture("/dev/video_cam", cv2.CAP_V4L2)
# #         self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
# #         self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1200)
# #         self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
# #         self.cap.set(cv2.CAP_PROP_FPS, 30)

# #         width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
# #         height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
# #         fps = self.cap.get(cv2.CAP_PROP_FPS)
# #         print(f"Width: {width}, Height: {height}, FPS: {fps}")

# #         # Publishers (JSON과 Path 제거)
# #         self.pose_publishers = {}
# #         self.odom_publishers = {}
        
# #         # Data storage
# #         self.robot_states = {}
        
# #         # TF broadcaster
# #         self.tf_broadcaster = TransformBroadcaster(self)
# #         self.velocity_alpha = 0.7
# #         self.odom_frame = "odom"
# #         self.map_frame = "map"

# #     def update_robot_state(self, marker_id, x, y, yaw, current_time):
# #         """로봇 상태 업데이트 및 속도 계산"""
# #         if marker_id not in self.robot_states:
# #             self.robot_states[marker_id] = RobotState()

# #         state = self.robot_states[marker_id]
        
# #         if not state.initialized:
# #             state.x = x
# #             state.y = y
# #             state.yaw = yaw
# #             state.last_x = x
# #             state.last_y = y
# #             state.last_yaw = yaw
# #             state.last_time = current_time
# #             state.initialized = True
# #             return

# #         dt = current_time - state.last_time
# #         if dt <= 0:
# #             return

# #         dx = x - state.last_x
# #         dy = y - state.last_y
# #         dyaw = normalize_angle(yaw - state.last_yaw)

# #         vx_new = dx / dt
# #         vy_new = dy / dt
# #         vyaw_new = dyaw / dt

# #         state.vx = self.velocity_alpha * state.vx + (1 - self.velocity_alpha) * vx_new
# #         state.vy = self.velocity_alpha * state.vy + (1 - self.velocity_alpha) * vy_new
# #         state.vyaw = self.velocity_alpha * state.vyaw + (1 - self.velocity_alpha) * vyaw_new

# #         state.x = x
# #         state.y = y
# #         state.yaw = yaw
# #         state.last_x = x
# #         state.last_y = y
# #         state.last_yaw = yaw
# #         state.last_time = current_time



# #     def create_odometry_message(self, marker_id, current_time):
# #         """Nav2 호환 Odometry 메시지 생성"""
# #         if marker_id not in self.robot_states:
# #             return None

# #         state = self.robot_states[marker_id]
# #         if not state.initialized:
# #             return None

# #         odom = Odometry()
# #         odom.header.stamp = self.get_clock().now().to_msg()
# #         odom.header.frame_id = "odom"
# #         odom.child_frame_id = f"base_footprint_{marker_id}"

# #         odom.pose.pose.position.x = state.x
# #         odom.pose.pose.position.y = state.y
# #         odom.pose.pose.position.z = 0.0

# #         q = yaw_to_quaternion(state.yaw)
# #         odom.pose.pose.orientation.x = q['x']
# #         odom.pose.pose.orientation.y = q['y']
# #         odom.pose.pose.orientation.z = q['z']
# #         odom.pose.pose.orientation.w = q['w']

# #         cos_yaw = math.cos(state.yaw)
# #         sin_yaw = math.sin(state.yaw)
        
# #         vx_body = cos_yaw * state.vx + sin_yaw * state.vy
# #         vy_body = -sin_yaw * state.vx + cos_yaw * state.vy
        
# #         odom.twist.twist.linear.x = vx_body
# #         odom.twist.twist.linear.y = vy_body
# #         odom.twist.twist.linear.z = 0.0
# #         odom.twist.twist.angular.x = 0.0
# #         odom.twist.twist.angular.y = 0.0
# #         odom.twist.twist.angular.z = state.vyaw

# #         pose_cov = [
# #             0.01, 0.0,  0.0,    0.0,    0.0,    0.0,
# #             0.0,  0.01, 0.0,    0.0,    0.0,    0.0,
# #             0.0,  0.0,  1e6,    0.0,    0.0,    0.0,
# #             0.0,  0.0,  0.0,    1e6,    0.0,    0.0,
# #             0.0,  0.0,  0.0,    0.0,    1e6,    0.0,
# #             0.0,  0.0,  0.0,    0.0,    0.0,    0.02
# #         ]
        
# #         twist_cov = [
# #             0.02, 0.0,  0.0,    0.0,    0.0,    0.0,
# #             0.0,  0.02, 0.0,    0.0,    0.0,    0.0,
# #             0.0,  0.0,  1e6,    0.0,    0.0,    0.0,
# #             0.0,  0.0,  0.0,    1e6,    0.0,    0.0,
# #             0.0,  0.0,  0.0,    0.0,    1e6,    0.0,
# #             0.0,  0.0,  0.0,    0.0,    0.0,    0.05
# #         ]

# #         odom.pose.covariance = pose_cov
# #         odom.twist.covariance = twist_cov

# #         return odom

# #     def process_frame(self):
# #         start_time = time.time()
        
# #         ret, frame = self.cap.read()
# #         if not ret:
# #             self.get_logger().warn("Camera frame not received!")
# #             return

# #         detect_start = time.time()
# #         current_time = time.time()
# #         detected = self.detector.detect_markers(frame)
# #         detect_end = time.time()
# #         detect_time_ms = (detect_end - detect_start) * 1000

# #         processing_start = time.time()
        
# #         for marker in detected:
# #             marker_id = marker["id"]
# #             x, y = marker["robot_xy"]
# #             yaw = marker["yaw"]
# #             cx, cy = marker["text_pos"]

# #             self.update_robot_state(marker_id, x, y, yaw, current_time)

# #             # Publishers 생성 (처음 감지될 때)
# #             if marker_id not in self.pose_publishers:
# #                 self.pose_publishers[marker_id] = self.create_publisher(
# #                     PoseStamped, f"/robot{marker_id}/pose", 10)
# #                 self.odom_publishers[marker_id] = self.create_publisher(
# #                     Odometry, f"/odom_{marker_id}", 10)

# #             # PoseStamped 생성
# #             pose = PoseStamped()
# #             pose.header.stamp = self.get_clock().now().to_msg()
# #             pose.header.frame_id = self.map_frame
# #             pose.pose.position.x = x
# #             pose.pose.position.y = y
# #             pose.pose.position.z = 0.0

# #             q = yaw_to_quaternion(yaw)
# #             pose.pose.orientation.x = q['x']
# #             pose.pose.orientation.y = q['y']
# #             pose.pose.orientation.z = q['z']
# #             pose.pose.orientation.w = q['w']

# #             # Pose 발행 (실시간 위치 추적용)
# #             self.pose_publishers[marker_id].publish(pose)

# #             # Odometry 발행
# #             odom = self.create_odometry_message(marker_id, current_time)
# #             if odom is not None:
# #                 odom.header.frame_id = f"odom_{marker_id}"
# #                 odom.child_frame_id = f"base_link_{marker_id}"
# #                 self.odom_publishers[marker_id].publish(odom)

# #         processing_end = time.time()
# #         processing_time_ms = (processing_end - processing_start) * 1000

# #         display_start = time.time()
# #         cv2.imshow("Aruco Odometry Publisher", frame)
# #         cv2.waitKey(1)
# #         display_end = time.time()
# #         display_time_ms = (display_end - display_start) * 1000

# #         end_time = time.time()
# #         total_time_ms = (end_time - start_time) * 1000

# #         # 성능 정보 출력 (원본 코드와 동일)
# #         if len(detected) > 0:  # 마커가 감지된 경우만 출력
# #             print(f"=== process_frame 성능 측정 ===")
# #             print(f"마커 감지: {detect_time_ms:.2f} ms")
# #             print(f"마커 처리: {processing_time_ms:.2f} ms")
# #             print(f"화면 출력: {display_time_ms:.2f} ms")
# #             print(f"전체 시간: {total_time_ms:.2f} ms ({len(detected)}개 마커)")
# #             print(f"FPS: {1000/total_time_ms:.1f}")
# #             print("=" * 30)

# #     def destroy_node(self):
# #         self.cap.release()
# #         cv2.destroyAllWindows()
# #         super().destroy_node()


# # def main():
# #     rclpy.init()
# #     base_path = '/home/addinnedu/monitoring_camera_ws/src/aruco_marker_pkg/include/'
# #     camera_matrix = np.load(base_path + 'camera_matrix.npy')
# #     dist_coeffs = np.load(base_path + 'dist_coeffs.npy')

# #     node = ArucoOdometryPublisher(camera_matrix, dist_coeffs)
# #     try:
# #         while rclpy.ok():
# #             node.process_frame()
# #             rclpy.spin_once(node, timeout_sec=0.005)
# #     except KeyboardInterrupt:
# #         pass
# #     finally:
# #         node.destroy_node()
# #         rclpy.shutdown()


# # if __name__ == '__main__':
# #     main()

# #!/usr/bin/env python3
# import cv2
# import numpy as np
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped
# from nav_msgs.msg import Odometry
# import math
# import time

# from tf2_ros import TransformBroadcaster
# from geometry_msgs.msg import TransformStamped


# def yaw_to_quaternion(yaw):
#     """Z축 회전(yaw, radian)을 쿼터니언(x, y, z, w)으로 변환."""
#     q = {}
#     q['x'] = 0.0
#     q['y'] = 0.0
#     q['z'] = math.sin(yaw / 2.0)
#     q['w'] = math.cos(yaw / 2.0)
#     return q


# def normalize_angle(angle):
#     """각도를 -π ~ π 범위로 정규화"""
#     while angle > math.pi:
#         angle -= 2.0 * math.pi
#     while angle < -math.pi:
#         angle += 2.0 * math.pi
#     return angle


# class ArucoDetector:
#     def __init__(self, camera_matrix, dist_coeffs, marker_length=0.05):
#         self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
#         self.aruco_params = cv2.aruco.DetectorParameters_create()
#         self.camera_matrix = camera_matrix
#         self.dist_coeffs = dist_coeffs
#         self.marker_length = marker_length

#         self.src_pts = np.array([
#             [-1.420,  0.097],  
#             [-1.528, -0.830],  
#             [ 0.459,  0.076],  
#             [ 0.410, -0.845]
#         ], dtype=np.float32)
#         self.dst_pts = np.array([
#             [0.05, 0.05],
#             [0.05, 0.95],
#             [1.95, 0.05],
#             [1.95, 0.95]
#         ], dtype=np.float32)
#         self.H = cv2.getPerspectiveTransform(self.src_pts, self.dst_pts)

#     def transform_point(self, pt):
#         src = np.array([pt[0], pt[1], 1.0], dtype=np.float32)
#         dst = self.H @ src
#         dst = dst / dst[2]
#         return float(dst[0]), float(dst[1])

#     def detect_markers(self, frame):
#         corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)
#         detected = []
#         if ids is not None:
#             rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
#                 corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
#             for i in range(len(ids)):
#                 marker_id = int(ids[i][0])
#                 tvec = tvecs[i][0]
#                 rvec = rvecs[i][0]
#                 x, y = self.transform_point((tvec[0], tvec[1]))
#                 x, y = round(x, 2), round(y, 2)

#                 rot_mat, _ = cv2.Rodrigues(rvec)
#                 yaw = -(np.arctan2(rot_mat[1, 0], rot_mat[0, 0]) - np.pi / 2)
#                 yaw = normalize_angle(yaw)
#                 yaw = round(float(yaw), 3)

#                 corners_reshaped = corners[i].reshape((4, 2))
#                 center_x = int(np.mean(corners_reshaped[:, 0]))
#                 center_y = int(np.mean(corners_reshaped[:, 1]))

#                 detected.append({
#                     "id": marker_id,
#                     "robot_xy": [x, y],
#                     "yaw": yaw,
#                     "text_pos": (center_x, center_y)
#                 })
        
#         return detected


# class RobotState:
#     """각 로봇의 상태를 저장하는 클래스"""
#     def __init__(self):
#         self.x = 0.0
#         self.y = 0.0
#         self.yaw = 0.0
#         self.vx = 0.0
#         self.vy = 0.0
#         self.vyaw = 0.0
#         self.last_time = None
#         self.last_x = 0.0
#         self.last_y = 0.0
#         self.last_yaw = 0.0
#         self.initialized = False


# class ArucoOdometryPublisher(Node):
#     def __init__(self, camera_matrix, dist_coeffs):
#         super().__init__('aruco_odometry_publisher')
#         self.detector = ArucoDetector(camera_matrix, dist_coeffs)
#         self.cap = cv2.VideoCapture("/dev/video_cam", cv2.CAP_V4L2)
#         self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
#         self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1200)
#         self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
#         self.cap.set(cv2.CAP_PROP_FPS, 30)

#         width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
#         height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
#         fps = self.cap.get(cv2.CAP_PROP_FPS)
#         print(f"Width: {width}, Height: {height}, FPS: {fps}")

#         # Publishers (JSON과 Path 제거)
#         self.pose_publishers = {}
#         self.odom_publishers = {}
        
#         # Data storage
#         self.robot_states = {}
        
#         # TF broadcaster
#         self.tf_broadcaster = TransformBroadcaster(self)
#         self.velocity_alpha = 0.3
#         self.odom_frame = "odom"
#         self.map_frame = "map"

#     def update_robot_state(self, marker_id, x, y, yaw, current_time):
#         """로봇 상태 업데이트 및 속도 계산"""
#         if marker_id not in self.robot_states:
#             self.robot_states[marker_id] = RobotState()

#         state = self.robot_states[marker_id]
        
#         if not state.initialized:
#             state.x = x
#             state.y = y
#             state.yaw = yaw
#             state.last_x = x
#             state.last_y = y
#             state.last_yaw = yaw
#             state.last_time = current_time
#             state.initialized = True
#             return

#         dt = current_time - state.last_time
#         if dt <= 0:
#             return

#         dx = x - state.last_x
#         dy = y - state.last_y
#         dyaw = normalize_angle(yaw - state.last_yaw)

#         vx_new = dx / dt
#         vy_new = dy / dt
#         vyaw_new = dyaw / dt

#         state.vx = self.velocity_alpha * state.vx + (1 - self.velocity_alpha) * vx_new
#         state.vy = self.velocity_alpha * state.vy + (1 - self.velocity_alpha) * vy_new
#         state.vyaw = self.velocity_alpha * state.vyaw + (1 - self.velocity_alpha) * vyaw_new

#         state.x = x
#         state.y = y
#         state.yaw = yaw
#         state.last_x = x
#         state.last_y = y
#         state.last_yaw = yaw
#         state.last_time = current_time



#     def create_odometry_message(self, marker_id, current_time):
#         """Nav2 호환 Odometry 메시지 생성"""
#         if marker_id not in self.robot_states:
#             return None

#         state = self.robot_states[marker_id]
#         if not state.initialized:
#             return None

#         odom = Odometry()
#         odom.header.stamp = self.get_clock().now().to_msg()
#         odom.header.frame_id = "odom"
#         odom.child_frame_id = f"base_footprint_{marker_id}"

#         odom.pose.pose.position.x = state.x
#         odom.pose.pose.position.y = state.y
#         odom.pose.pose.position.z = 0.0

#         q = yaw_to_quaternion(state.yaw)
#         odom.pose.pose.orientation.x = q['x']
#         odom.pose.pose.orientation.y = q['y']
#         odom.pose.pose.orientation.z = q['z']
#         odom.pose.pose.orientation.w = q['w']

#         cos_yaw = math.cos(state.yaw)
#         sin_yaw = math.sin(state.yaw)
        
#         vx_body = cos_yaw * state.vx + sin_yaw * state.vy
#         vy_body = -sin_yaw * state.vx + cos_yaw * state.vy
        
#         odom.twist.twist.linear.x = vx_body
#         odom.twist.twist.linear.y = vy_body
#         odom.twist.twist.linear.z = 0.0
#         odom.twist.twist.angular.x = 0.0
#         odom.twist.twist.angular.y = 0.0
#         odom.twist.twist.angular.z = state.vyaw

#         pose_cov = [
#             0.01, 0.0,  0.0,    0.0,    0.0,    0.0,
#             0.0,  0.01, 0.0,    0.0,    0.0,    0.0,
#             0.0,  0.0,  1e6,    0.0,    0.0,    0.0,
#             0.0,  0.0,  0.0,    1e6,    0.0,    0.0,
#             0.0,  0.0,  0.0,    0.0,    1e6,    0.0,
#             0.0,  0.0,  0.0,    0.0,    0.0,    0.02
#         ]
        
#         twist_cov = [
#             0.02, 0.0,  0.0,    0.0,    0.0,    0.0,
#             0.0,  0.02, 0.0,    0.0,    0.0,    0.0,
#             0.0,  0.0,  1e6,    0.0,    0.0,    0.0,
#             0.0,  0.0,  0.0,    1e6,    0.0,    0.0,
#             0.0,  0.0,  0.0,    0.0,    1e6,    0.0,
#             0.0,  0.0,  0.0,    0.0,    0.0,    0.05
#         ]

#         odom.pose.covariance = pose_cov
#         odom.twist.covariance = twist_cov

#         return odom

#     def process_frame(self):
#         start_time = time.time()
        
#         ret, frame = self.cap.read()
#         if not ret:
#             self.get_logger().warn("Camera frame not received!")
#             return

#         detect_start = time.time()
#         current_time = time.time()
#         detected = self.detector.detect_markers(frame)
#         detect_end = time.time()
#         detect_time_ms = (detect_end - detect_start) * 1000

#         processing_start = time.time()
        
#         for marker in detected:
#             marker_id = marker["id"]
#             x, y = marker["robot_xy"]
#             yaw = marker["yaw"]
#             cx, cy = marker["text_pos"]

#             self.update_robot_state(marker_id, x, y, yaw, current_time)

#             # Publishers 생성 (처음 감지될 때)
#             if marker_id not in self.pose_publishers:
#                 self.pose_publishers[marker_id] = self.create_publisher(
#                     PoseStamped, f"/robot{marker_id}/pose", 10)
#                 self.odom_publishers[marker_id] = self.create_publisher(
#                     Odometry, f"/odom_{marker_id}", 10)

#             # PoseStamped 생성
#             pose = PoseStamped()
#             pose.header.stamp = self.get_clock().now().to_msg()
#             pose.header.frame_id = self.map_frame
#             pose.pose.position.x = x
#             pose.pose.position.y = y
#             pose.pose.position.z = 0.0

#             q = yaw_to_quaternion(yaw)
#             pose.pose.orientation.x = q['x']
#             pose.pose.orientation.y = q['y']
#             pose.pose.orientation.z = q['z']
#             pose.pose.orientation.w = q['w']

#             # Pose 발행 (실시간 위치 추적용)
#             self.pose_publishers[marker_id].publish(pose)

#             # Odometry 발행
#             odom = self.create_odometry_message(marker_id, current_time)
#             if odom is not None:
#                 odom.header.frame_id = f"odom_{marker_id}"
#                 odom.child_frame_id = f"base_link_{marker_id}"
#                 self.odom_publishers[marker_id].publish(odom)

#         processing_end = time.time()
#         processing_time_ms = (processing_end - processing_start) * 1000

#         display_start = time.time()
#         cv2.imshow("Aruco Odometry Publisher", frame)
#         cv2.waitKey(1)
#         display_end = time.time()
#         display_time_ms = (display_end - display_start) * 1000

#         end_time = time.time()
#         total_time_ms = (end_time - start_time) * 1000

#         # 성능 정보 출력 (원본 코드와 동일)
#         if len(detected) > 0:  # 마커가 감지된 경우만 출력
#             print(f"=== process_frame 성능 측정 ===")
#             print(f"마커 감지: {detect_time_ms:.2f} ms")
#             print(f"마커 처리: {processing_time_ms:.2f} ms")
#             print(f"화면 출력: {display_time_ms:.2f} ms")
#             print(f"전체 시간: {total_time_ms:.2f} ms ({len(detected)}개 마커)")
#             print(f"FPS: {1000/total_time_ms:.1f}")
#             print("=" * 30)

#     def destroy_node(self):
#         self.cap.release()
#         cv2.destroyAllWindows()
#         super().destroy_node()


# def main():
#     rclpy.init()
#     base_path = '/home/addinnedu/monitoring_camera_ws/src/aruco_marker_pkg/include/'
#     camera_matrix = np.load(base_path + 'camera_matrix.npy')
#     dist_coeffs = np.load(base_path + 'dist_coeffs.npy')

#     node = ArucoOdometryPublisher(camera_matrix, dist_coeffs)
#     try:
#         while rclpy.ok():
#             node.process_frame()
#             rclpy.spin_once(node, timeout_sec=0.005)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
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

        # Publishers (JSON과 Path 제거)
        self.pose_publishers = {}
        self.odom_publishers = {}
        
        # Data storage
        self.robot_states = {}
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Low pass filter 계수들
        self.velocity_alpha = 0.3    # 속도용 필터 계수
        self.position_alpha = 0.7    # 위치용 필터 계수 (위치는 속도보다 덜 민감하게)
        self.yaw_alpha = 0.6         # yaw용 필터 계수
        
        self.odom_frame = "odom"
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

    def create_odometry_message(self, marker_id, current_time):
        """Nav2 호환 Odometry 메시지 생성"""
        if marker_id not in self.robot_states:
            return None

        state = self.robot_states[marker_id]
        if not state.initialized:
            return None

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
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

        pose_cov = [
            0.01, 0.0,  0.0,    0.0,    0.0,    0.0,
            0.0,  0.01, 0.0,    0.0,    0.0,    0.0,
            0.0,  0.0,  1e6,    0.0,    0.0,    0.0,
            0.0,  0.0,  0.0,    1e6,    0.0,    0.0,
            0.0,  0.0,  0.0,    0.0,    1e6,    0.0,
            0.0,  0.0,  0.0,    0.0,    0.0,    0.02
        ]
        
        twist_cov = [
            0.02, 0.0,  0.0,    0.0,    0.0,    0.0,
            0.0,  0.02, 0.0,    0.0,    0.0,    0.0,
            0.0,  0.0,  1e6,    0.0,    0.0,    0.0,
            0.0,  0.0,  0.0,    1e6,    0.0,    0.0,
            0.0,  0.0,  0.0,    0.0,    1e6,    0.0,
            0.0,  0.0,  0.0,    0.0,    0.0,    0.05
        ]

        odom.pose.covariance = pose_cov
        odom.twist.covariance = twist_cov

        return odom

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
            cx, cy = marker["text_pos"]

            # 원본 데이터를 필터링하여 상태 업데이트
            self.update_robot_state(marker_id, x, y, yaw, current_time)

            # Publishers 생성 (처음 감지될 때)
            if marker_id not in self.pose_publishers:
                self.pose_publishers[marker_id] = self.create_publisher(
                    PoseStamped, f"/robot{marker_id}/pose", 10)
                self.odom_publishers[marker_id] = self.create_publisher(
                    Odometry, f"/odom_{marker_id}", 10)

            # 필터링된 상태값 사용
            state = self.robot_states[marker_id]
            
            # PoseStamped 생성 (필터링된 값 사용)
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = self.map_frame
            pose.pose.position.x = state.x
            pose.pose.position.y = state.y
            pose.pose.position.z = 0.0

            q = yaw_to_quaternion(state.yaw)
            pose.pose.orientation.x = q['x']
            pose.pose.orientation.y = q['y']
            pose.pose.orientation.z = q['z']
            pose.pose.orientation.w = q['w']

            # Pose 발행 (실시간 위치 추적용)
            self.pose_publishers[marker_id].publish(pose)

            # Odometry 발행
            odom = self.create_odometry_message(marker_id, current_time)
            if odom is not None:
                odom.header.frame_id = f"odom_{marker_id}"
                odom.child_frame_id = f"base_link_{marker_id}"
                self.odom_publishers[marker_id].publish(odom)

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
            rclpy.spin_once(node, timeout_sec=0.005)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()