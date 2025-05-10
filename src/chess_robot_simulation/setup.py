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
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Chess robot arm simulation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'chess_robot_node = chess_robot_simulation.chess_robot_node:main',
        'minimal_node = chess_robot_simulation.minimal_node:main',
        'minimal_publisher = chess_robot_simulation.minimal_publisher:main',

        ],
    },
)