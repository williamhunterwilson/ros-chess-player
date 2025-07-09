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
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'geometry_msgs',
        'std_msgs',
        'std_srvs',
        'example_interfaces',
        'visualization_msgs',
        'sensor_msgs',
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
            'piece_attachment_manager = chess_robot_simulation.piece_attachment_manager:main',
            'chess_move_client = chess_robot_simulation.chess_move_client:main',
            'gripper_control = chess_robot_simulation.gripper_control:main',
            'joint_state_publisher = chess_robot_simulation.joint_state_publisher:main',
            'robot_description_publisher = chess_robot_simulation.robot_description_publisher:main',
            'simple_chess_arm = chess_robot_simulation.simple_chess_arm:main',
            'test_arm_movement = chess_robot_simulation.test_arm_movement:main',
            'test_robot_movement = chess_robot_simulation.test_robot_movement:main',
            'debug_joint_publisher = chess_robot_simulation.debug_joint_publisher:main',
            'chess_visualization = chess_robot_simulation.chess_visualization:main',
            'simple_chess_client = chess_robot_simulation.simple_chess_client:main',
        ],
    },
)
