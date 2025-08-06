#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class ArucoDetector:
    def __init__(self, camera_matrix, dist_coeffs, marker_length=0.05, axis_length=0.05):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.marker_length = marker_length
        self.axis_length = axis_length

        self.src_pts = np.array([
            [-0.96,  0.41],
            [-0.98, -0.49],
            [ 0.97,  0.39],
            [ 0.99, -0.53]
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

    def detect_and_annotate(self, frame):
        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)
        detected_markers = []
        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
            for i, corner in enumerate(corners):
                marker_id = int(ids[i][0])
                tvec = tvecs[i][0]
                rvec = rvecs[i][0]
                robot_x, robot_y = self.transform_point((tvec[0], tvec[1]))
                robot_x = round(robot_x, 2)
                robot_y = round(robot_y, 2)

                # --- yaw(라디안) 계산 ---
                rot_mat, _ = cv2.Rodrigues(rvec)
                yaw = np.arctan2(rot_mat[1,0], rot_mat[0,0])
                yaw = yaw - np.pi/2  # 윗방향(↑)이 yaw=0
                # -pi ~ pi로 wrap
                if yaw > np.pi:
                    yaw -= 2*np.pi
                elif yaw < -np.pi:
                    yaw += 2*np.pi
                yaw = round(float(yaw), 3)

                detected_markers.append({
                    "id": marker_id,
                    "robot_xy": [robot_x, robot_y],
                    "yaw": yaw
                })

                # 화면 표시: 두 줄로!
                corners_reshaped = corner.reshape((4, 2))
                center_x = int(np.mean(corners_reshaped[:, 0]))
                center_y = int(np.mean(corners_reshaped[:, 1]))
                cv2.putText(
                    frame,
                    f"RobotXY: ({robot_x:.2f}, {robot_y:.2f})",
                    (center_x + 10, center_y + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 128, 255), 2)
                cv2.putText(
                    frame,
                    f"Yaw: {yaw:.2f}",
                    (center_x + 10, center_y + 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 128, 255), 2)
        return frame, detected_markers

class MarkerPublisherNode(Node):
    def __init__(self, camera_matrix, dist_coeffs):
        super().__init__('aruco_marker_publisher')
        self.publisher = self.create_publisher(String, 'detected_markers', 10)
        self.detector = ArucoDetector(camera_matrix, dist_coeffs)
        self.cap = cv2.VideoCapture("/dev/video_cam")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    def process(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Camera frame not received!")
            return
        frame, detected_markers = self.detector.detect_and_annotate(frame)
        msg = String()
        msg.data = json.dumps(detected_markers)
        self.publisher.publish(msg)
        cv2.imshow("Aruco Detection", frame)
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

    node = MarkerPublisherNode(camera_matrix, dist_coeffs)

    try:
        while rclpy.ok():
            node.process()
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
