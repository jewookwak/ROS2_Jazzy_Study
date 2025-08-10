from setuptools import find_packages, setup

package_name = 'pid_controller_node'

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
    maintainer='pinky',
    maintainer_email='pinky@todo.todo',
    description='PID velocity filtering node using odometry feedback',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_velocity_filter_node = pid_controller_node.pid_velocity_filter_node:main',
            'move_to_goal = pid_controller_node.move_to_goal:main',
            'move_to_goal_tuning = pid_controller_node.move_to_goal_tuning:main',
            'move_to_goal_pid = pid_controller_node.move_to_goal_pid:main',
        ],
    },
)
