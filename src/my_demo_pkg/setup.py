# ~/ros2_ws/src/my_demo_pkg/setup.py

from setuptools import setup

package_name = 'my_demo_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your name',
    maintainer_email='your@email.com',
    description='A simple demo package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker1 = my_demo_pkg.talker1:main',
            'talker2 = my_demo_pkg.talker2:main',
        ],
    },
)
