#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import math

class ArmMovementTester(Node):
    def __init__(self):
        super().__init__('arm_movement_tester')
        
        # Publisher for joint states
        self.joint_publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # Joint names
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        self.get_logger().info('Arm Movement Tester Started')
        
    def publish_joint_position(self, positions):
        """Publish specific joint positions"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = positions
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        
        self.joint_publisher.publish(msg)
        self.get_logger().info(f'Published joints: {[f"{p:.2f}" for p in positions]}')
    
    def test_arm_movement(self):
        """Test various arm positions"""
        self.get_logger().info('Starting arm movement test...')
        
        # Test positions
        positions = [
            ([0.0, -1.0, -1.2, -1.3, 0.0, 0.0], "Home position"),
            ([0.5, -0.8, -1.0, -1.5, 0.0, 0.0], "Chess ready position"),
            ([1.0, -1.2, -0.8, -1.0, 0.0, 0.0], "Reaching right"),
            ([-0.5, -1.5, -1.0, -0.8, 0.0, 0.0], "Reaching left"),
            ([0.0, -0.5, -1.8, -1.2, 0.0, 0.0], "High reach"),
            ([0.0, -1.0, -1.2, -1.3, 0.0, 0.0], "Back to home"),
        ]
        
        for joints, description in positions:
            self.get_logger().info(f'Moving to: {description}')
            self.publish_joint_position(joints)
            time.sleep(2.0)  # Wait 2 seconds between moves
        
        self.get_logger().info('Arm movement test completed!')

def main(args=None):
    rclpy.init(args=args)
    tester = ArmMovementTester()
    
    try:
        # Run the test
        tester.test_arm_movement()
        
        # Keep publishing the last position
        while rclpy.ok():
            tester.publish_joint_position([0.0, -1.0, -1.2, -1.3, 0.0, 0.0])
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print('\nTest stopped by user')
    
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 