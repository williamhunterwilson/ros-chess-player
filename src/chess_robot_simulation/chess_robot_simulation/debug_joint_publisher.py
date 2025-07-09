#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import math

class DebugJointPublisher(Node):
    def __init__(self):
        super().__init__('debug_joint_publisher')
        
        # Publisher for joint states
        self.joint_publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # Joint names (must match URDF)
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Current joint positions
        self.joint_positions = [0.0, -1.0, -1.2, -1.3, 0.0, 0.0]
        
        # Timer to publish joint states
        self.create_timer(0.1, self.publish_joint_states)
        
        # Timer to move the arm
        self.create_timer(3.0, self.move_arm)
        
        self.step = 0
        self.get_logger().info('Debug Joint Publisher started - Robot arm should move!')
    
    def publish_joint_states(self):
        """Publish current joint states"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        
        self.joint_publisher.publish(msg)
    
    def move_arm(self):
        """Move arm through different positions"""
        positions = [
            [0.0, -1.0, -1.2, -1.3, 0.0, 0.0],  # Home
            [0.5, -0.8, -1.0, -1.5, 0.0, 0.0],  # Right
            [1.0, -1.2, -0.8, -1.0, 0.0, 0.0],  # Far right
            [-0.5, -1.5, -1.0, -0.8, 0.0, 0.0], # Left
            [0.0, -0.5, -1.8, -1.2, 0.0, 0.0],  # High
            [0.0, -1.0, -1.2, -1.3, 0.0, 0.0],  # Back home
        ]
        
        names = ["Home", "Right", "Far Right", "Left", "High", "Back Home"]
        
        if self.step < len(positions):
            self.joint_positions = positions[self.step]
            self.get_logger().info(f'Moving to: {names[self.step]} - {self.joint_positions}')
            self.step += 1
        else:
            self.step = 0  # Loop back

def main(args=None):
    rclpy.init(args=args)
    publisher = DebugJointPublisher()
    
    print("=" * 50)
    print("DEBUG: Robot arm should start moving in RViz!")
    print("If the arm doesn't move, there's an issue with:")
    print("1. joint_states topic not being published")
    print("2. robot_state_publisher not running")
    print("3. RViz not showing the robot model")
    print("=" * 50)
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        print('\nDebug stopped by user')
    
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 