from setuptools import setup
import os
from glob import glob

package_name = 'chess_robot_simulation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'geometry_msgs',
        'std_msgs',
        'visualization_msgs',
        'moveit_py',
        'numpy',
        'python-chess',
    ],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Chess robot arm simulation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chess_robot_node = chess_robot_simulation.chess_robot_node:main',
        ],
    },
)
