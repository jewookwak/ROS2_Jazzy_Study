from setuptools import find_packages, setup

package_name = 'aruco_marker_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='addinnedu',
    maintainer_email='addinnedu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_publisher = aruco_marker_pkg.pose_publisher:main',
            'marker_publisher_node = aruco_marker_pkg.aruco_detector:main',
            'aruco_path_tracker = aruco_marker_pkg.aruco_path_tracker:main',
            'aruco_TF_broadcaster = aruco_marker_pkg.aruco_TF_broadcaster:main',
            'aruco_odom_publisher = aruco_marker_pkg.aruco_odom_publisher:main',
            'multi_aruco_odom_publisher = aruco_marker_pkg.multi_aruco_odom_publisher:main',
            'monitoring_map_gui = aruco_marker_pkg.monitoring_map_gui:main',
            'aruco_test = aruco_marker_pkg.aruco_test:main',
            'monitoring_gui = aruco_marker_pkg.monitoring_gui:main',
            'monitoring_gui_matplot = aruco_marker_pkg.monitoring_gui_matplot:main',
        ],
    },
)