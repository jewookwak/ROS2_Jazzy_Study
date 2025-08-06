#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from scipy.spatial.transform import Rotation as R_
import time

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.publisher = self.create_publisher(String, 'detected_markers', 10)

    def publish_markers(self, marker_list):
        msg = String()
        msg.data = json.dumps(marker_list)
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = MarkerPublisher()

    dev_num = "/dev/video_cam"   # 필요하면 0, 1, 2 등으로 바꿔서 테스트하세요
    cap = cv2.VideoCapture(dev_num)
    if not cap.isOpened():
        print(f"ERROR: Video device {dev_num} cannot be opened!")
        exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    base_path = '/home/addinnedu/monitoring_camera_ws/src/aruco_marker_pkg/include/'

    # --- 파일 로딩 후 shape 출력 (디버깅용) ---
    camera_matrix = np.load(base_path + 'camera_matrix.npy')
    print("camera_matrix shape:", camera_matrix.shape)
    dist_coeffs = np.load(base_path + 'dist_coeffs.npy')
    print("dist_coeffs shape:", dist_coeffs.shape)
    R = np.load(base_path + 'R_cam_to_table.npy')
    print("R shape:", R.shape)
    T = np.load(base_path + 'T_cam_to_table.npy')
    print("T shape:", T.shape)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    parameters = cv2.aruco.DetectorParameters()
    marker_size = 50
    marker_3d_edges = np.array([
        [0, 0, 0], [0, marker_size, 0],
        [marker_size, marker_size, 0], [marker_size, 0, 0]
    ], dtype='float64').reshape((4, 1, 3))

    try:
        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                print("Camera frame not received")
                time.sleep(0.05)
                continue

            frame_undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs)
            corners, ids, _ = cv2.aruco.detectMarkers(
                frame_undistorted, aruco_dict, parameters=parameters)

            new_markers = []
            if ids is not None:
                for i, corner in enumerate(corners):
                    marker_id = int(ids[i][0])
                    corner = np.array(corner).reshape((4, 2))
                    ret_pnp, rvec, tvec = cv2.solvePnP(
                        marker_3d_edges, corner, camera_matrix, dist_coeffs
                    )
                    if ret_pnp:
                        trans = tvec.reshape(3, 1)
                        trans_applied = R @ trans + T
                        x = round(trans_applied[0][0] / 1000, 2)
                        y = round(trans_applied[1][0] / 1000, 2)
                        rot_mat_cam, _ = cv2.Rodrigues(rvec)
                        rot_mat_table = R @ rot_mat_cam
                        rot_obj = R_.from_matrix(rot_mat_table)
                        _, _, rz = rot_obj.as_euler('xyz', degrees=False)
                        rz = round(rz, 4)
                        new_markers.append({
                            "id": marker_id,
                            "x": x,
                            "y": y,
                            "rz": rz,
                        })
            node.publish_markers(new_markers)
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        cap.release()
        rclpy.shutdown()

if __name__ == '__main__':
    main()