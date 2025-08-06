#!/usr/bin/env python3
import numpy as np
import cv2
import os

# 원본 npy 파일 경로
source_path = '/home/addinnedu/monitoring_camera_ws/src/aruco_marker_pkg/include/'

try:
    # NumPy 파일 로드
    camera_matrix = np.load(os.path.join(source_path, 'camera_matrix.npy'))
    dist_coeffs = np.load(os.path.join(source_path, 'dist_coeffs.npy'))

    print("Camera Matrix:")
    print(camera_matrix)
    print("\nDistortion Coefficients:")
    print(dist_coeffs)

    # 현재 디렉토리에 OpenCV XML 형식으로 저장
    fs_camera = cv2.FileStorage('camera_matrix.xml', cv2.FILE_STORAGE_WRITE)
    fs_camera.write('camera_matrix', camera_matrix)
    fs_camera.release()

    fs_dist = cv2.FileStorage('dist_coeffs.xml', cv2.FILE_STORAGE_WRITE)
    fs_dist.write('dist_coeffs', dist_coeffs)
    fs_dist.release()

    print("\nConversion completed!")
    print("Created: camera_matrix.xml")
    print("Created: dist_coeffs.xml")
    print("Location:", os.getcwd())

except Exception as e:
    print(f"Error: {e}")
    print("Make sure the source files exist and you have the required packages.")
