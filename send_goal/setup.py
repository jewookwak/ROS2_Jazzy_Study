from setuptools import find_packages, setup

package_name = 'send_goal'

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
    maintainer='jewoo',
    maintainer_email='jewoo@todo.todo',
    description='Send FollowWaypoints action goals to Nav2',
    license='Apache-2.0',
    tests_require=['pytest'],
    
    entry_points={
        'console_scripts': [
            'simple_nav_goal = send_goal.simple_nav_goal:main',
            'python_node = send_goal.python_node:main',
            'follow_waypoints_client = send_goal.FollowWaypointsClient:main'
        ],
    },
)
