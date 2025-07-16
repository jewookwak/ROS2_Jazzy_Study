from setuptools import find_packages, setup

package_name = 'pinky_mapf'

setup(
    name=package_name,
    version='0.0.1',
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
    description='RMF client node for controlling robot via FollowWaypoints action',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follow_waypoints_client = pinky_mapf.follow_waypoints_client:main',
            'goal_point = pinky_mapf.goal_point:main',
            'goal_point1 = pinky_mapf.goal_point1:main',
            'goal_point2 = pinky_mapf.goal_point2:main',
            'goal_point3 = pinky_mapf.goal_point3:main',
        ],
    },
)
