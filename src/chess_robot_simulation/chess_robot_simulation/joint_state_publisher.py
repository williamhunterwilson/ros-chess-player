#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
import math

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        
        # Publisher for joint states
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # Joint names matching the URDF
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Initialize joint positions (extended ready position)
        self.joint_positions = [0.0, -1.0, -1.2, -1.3, 0.0, 0.0]  # Extended ready position
        
        # Create timer to publish joint states
        self.create_timer(0.1, self.publish_joint_states)  # 10 Hz
        
        self.get_logger().info('Joint State Publisher started')
    
    def publish_joint_states(self):
        """Publish current joint states"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = [0.0] * len(self.joint_names)  # Zero velocity for static display
        msg.effort = [0.0] * len(self.joint_names)    # Zero effort for static display
        
        self.joint_state_publisher.publish(msg)
    
    def update_joint_position(self, joint_name, position):
        """Update a specific joint position"""
        if joint_name in self.joint_names:
            index = self.joint_names.index(joint_name)
            self.joint_positions[index] = position
            self.get_logger().info(f'Updated {joint_name} to {position:.3f} radians')
        else:
            self.get_logger().warn(f'Unknown joint: {joint_name}')
    
    def set_home_position(self):
        """Set robot to home position"""
        self.joint_positions = [0.0, -1.0, -1.2, -1.3, 0.0, 0.0]
        self.get_logger().info('Robot set to home position')
    
    def set_chess_ready_position(self):
        """Set robot to chess-ready position (over the board)"""
        self.joint_positions = [0.5, -0.8, -1.0, -1.5, 0.0, 0.0]
        self.get_logger().info('Robot set to chess-ready position')

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 