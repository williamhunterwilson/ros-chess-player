import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('chess_robot_simulation')
    
    # Path to URDF file
    urdf_file = os.path.join(pkg_dir, 'urdf', 'chess_robot_arm.urdf')
    
    # Read URDF file
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # RViz config file
    rviz_config_file = os.path.join(pkg_dir, 'config', 'chess_robot.rviz')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation/Gazebo clock'
        ),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description
            }]
        ),
        
        # Chess robot node with arm
        Node(
            package='chess_robot_simulation',
            executable='chess_robot_with_arm',
            name='chess_robot_with_arm',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Static transform from world to robot base
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0.45', '0.2', '0', '0', '0', '0', 'world', 'base_link']
        ),
    ])
