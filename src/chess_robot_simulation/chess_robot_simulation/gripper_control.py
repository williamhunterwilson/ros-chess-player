#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import Bool

class GripperControlNode(Node):
    def __init__(self):
        super().__init__('gripper_control_node')
        
        # Service to control gripper
        self.gripper_service = self.create_service(
            SetBool,
            'gripper_control',
            self.handle_gripper_control
        )
        
        # Publisher to show gripper state
        self.gripper_state_publisher = self.create_publisher(Bool, 'gripper_state', 10)
        
        self.gripper_closed = False
        self.get_logger().info('Gripper Control Node Started')
        
        # Publish gripper state periodically
        self.create_timer(0.1, self.publish_gripper_state)
    
    def handle_gripper_control(self, request, response):
        """Handle gripper control requests"""
        if request.data:
            # Close gripper (pick up)
            self.gripper_closed = True
            self.get_logger().info('Gripper CLOSED - Picking up object')
            response.success = True
            response.message = "Gripper closed successfully"
        else:
            # Open gripper (release)
            self.gripper_closed = False
            self.get_logger().info('Gripper OPENED - Releasing object')
            response.success = True
            response.message = "Gripper opened successfully"
        
        return response
    
    def publish_gripper_state(self):
        """Publish current gripper state"""
        msg = Bool()
        msg.data = self.gripper_closed
        self.gripper_state_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GripperControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 