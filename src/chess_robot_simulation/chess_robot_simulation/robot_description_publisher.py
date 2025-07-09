#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
from ament_index_python.packages import get_package_share_directory

class RobotDescriptionPublisher(Node):
    def __init__(self):
        super().__init__('robot_description_publisher')
        
        # Publisher for robot description
        self.robot_description_publisher = self.create_publisher(
            String, 
            'robot_description', 
            rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        )
        
        # Also publish as a parameter for robot_state_publisher
        self.declare_parameter('robot_description', '')
        
        # Load and publish URDF
        self.load_and_publish_urdf()
        
        # Publish periodically to ensure it's available
        self.create_timer(1.0, self.publish_robot_description)
        
        self.get_logger().info('Robot Description Publisher started')
    
    def load_and_publish_urdf(self):
        """Load URDF file and publish it"""
        try:
            # Try to get package share directory
            try:
                package_share_directory = get_package_share_directory('chess_robot_simulation')
                urdf_path = os.path.join(package_share_directory, 'urdf', 'chess_robot.urdf')
            except:
                # Fallback to relative path if package not installed
                current_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
                urdf_path = os.path.join(current_dir, 'urdf', 'chess_robot.urdf')
            
            # Check if file exists
            if not os.path.exists(urdf_path):
                self.get_logger().error(f'URDF file not found at: {urdf_path}')
                return False
            
            # Read URDF file
            with open(urdf_path, 'r') as urdf_file:
                self.urdf_content = urdf_file.read()
            
            self.get_logger().info(f'Successfully loaded URDF from: {urdf_path}')
            
            # Set the parameter as well
            self.set_parameters([rclpy.parameter.Parameter('robot_description', 
                                                         rclpy.parameter.Parameter.Type.STRING, 
                                                         self.urdf_content)])
            
            self.publish_robot_description()
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to load URDF: {e}')
            return False
    
    def publish_robot_description(self):
        """Publish the robot description"""
        if hasattr(self, 'urdf_content'):
            msg = String()
            msg.data = self.urdf_content
            self.robot_description_publisher.publish(msg)
            self.get_logger().debug('Published robot description')

def main(args=None):
    rclpy.init(args=args)
    node = RobotDescriptionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 