#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts
import time
import math

class SimpleChessArm(Node):
    def __init__(self):
        super().__init__('simple_chess_arm')
        
        # Publisher for joint states
        self.joint_publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # Subscribe to chess moves
        self.move_subscription = self.create_subscription(
            String,
            'chess_move_cmd',
            self.handle_move_command,
            10)
        
        # Service for chess moves
        self.move_service = self.create_service(
            AddTwoInts,
            'make_chess_move',
            self.handle_move_service)
        
        # Joint names
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Current joint positions (start in extended position)
        self.current_joints = [0.0, -1.0, -1.2, -1.3, 0.0, 0.0]
        
        # Timer to continuously publish joint states
        self.create_timer(0.1, self.publish_joints)
        
        self.get_logger().info('Simple Chess Arm started - ready for moves!')
    
    def publish_joints(self):
        """Continuously publish joint states"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_joints
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        
        self.joint_publisher.publish(msg)
    
    def handle_move_command(self, msg):
        """Handle string move commands"""
        move_string = msg.data.strip()
        self.get_logger().info(f'Received move command: {move_string}')
        self.simulate_chess_move(move_string)
    
    def handle_move_service(self, request, response):
        """Handle service move commands"""
        from_square = request.a
        to_square = request.b
        
        # Convert to algebraic notation
        from_file = from_square % 8
        from_rank = from_square // 8
        to_file = to_square % 8
        to_rank = to_square // 8
        
        from_algebraic = chr(97 + from_file) + str(1 + from_rank)
        to_algebraic = chr(97 + to_file) + str(1 + to_rank)
        move_string = from_algebraic + to_algebraic
        
        self.get_logger().info(f'Received move service: {move_string}')
        success = self.simulate_chess_move(move_string)
        
        response.sum = 1 if success else 0
        return response
    
    def simulate_chess_move(self, move_string):
        """Simulate robot arm movement for a chess move"""
        self.get_logger().info(f'🤖 MOVING ARM for chess move: {move_string}')
        
        try:
            # Parse move (simple validation)
            if len(move_string) != 4:
                self.get_logger().error('Invalid move format')
                return False
            
            # Get from and to positions
            from_square = move_string[:2]
            to_square = move_string[2:]
            
            # Demonstrate arm movement sequence
            self.demonstrate_pick_and_place(from_square, to_square)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error during move: {e}')
            return False
    
    def demonstrate_pick_and_place(self, from_square, to_square):
        """Demonstrate pick and place movement"""
        self.get_logger().info(f'📍 Moving to pick up piece at {from_square}')
        
        # 1. Move to "approach" position (pointing toward from square)
        approach_joints = self.calculate_approach_position(from_square)
        self.move_to_joints(approach_joints, f"Approaching {from_square}")
        
        # 2. "Lower" to pick up piece
        pickup_joints = approach_joints.copy()
        pickup_joints[1] -= 0.3  # Lower shoulder lift
        self.move_to_joints(pickup_joints, "Lowering to pick up piece")
        
        # 3. "Lift" with piece
        self.move_to_joints(approach_joints, "Lifting piece")
        
        # 4. Move to destination
        self.get_logger().info(f'🎯 Moving to place piece at {to_square}')
        dest_joints = self.calculate_approach_position(to_square)
        self.move_to_joints(dest_joints, f"Moving to {to_square}")
        
        # 5. "Lower" to place piece
        place_joints = dest_joints.copy()
        place_joints[1] -= 0.3  # Lower shoulder lift
        self.move_to_joints(place_joints, "Lowering to place piece")
        
        # 6. Return to ready position
        self.move_to_joints(dest_joints, "Returning to ready position")
        
        self.get_logger().info(f'✅ Completed move: {from_square} to {to_square}')
    
    def calculate_approach_position(self, square):
        """Calculate joint positions to approach a chess square"""
        # Simple mapping: convert chess square to rough joint positions
        file_letter = square[0]  # a-h
        rank_number = int(square[1])  # 1-8
        
        # Convert to indices
        file_idx = ord(file_letter) - ord('a')  # 0-7
        rank_idx = rank_number - 1  # 0-7
        
        # Calculate shoulder pan angle based on file (a-h maps to left-right)
        # Center of board is around files d/e (indices 3/4)
        center_file = 3.5
        pan_angle = (file_idx - center_file) * 0.2  # Scale factor for movement
        
        # Calculate shoulder lift based on rank (1-8 maps to far-near)
        # Rank 1 is far (more negative lift), rank 8 is near (less negative lift)  
        base_lift = -1.0
        lift_angle = base_lift - (rank_idx - 4) * 0.1  # Adjust based on rank
        
        # Return joint configuration
        return [
            pan_angle,          # shoulder_pan_joint
            lift_angle,         # shoulder_lift_joint  
            -1.2,              # elbow_joint
            -1.3,              # wrist_1_joint
            0.0,               # wrist_2_joint
            0.0                # wrist_3_joint
        ]
    
    def move_to_joints(self, target_joints, description):
        """Smoothly move to target joint positions"""
        self.get_logger().info(f'   → {description}')
        
        start_joints = self.current_joints.copy()
        steps = 20  # Number of interpolation steps
        
        for step in range(steps + 1):
            alpha = step / steps
            
            # Interpolate between start and target
            for i in range(len(start_joints)):
                self.current_joints[i] = start_joints[i] + alpha * (target_joints[i] - start_joints[i])
            
            time.sleep(0.05)  # 50ms per step = 1 second total
        
        # Log final position
        joint_str = [f"{p:.2f}" for p in self.current_joints]
        self.get_logger().debug(f'     Final joints: {joint_str}')

def main(args=None):
    rclpy.init(args=args)
    
    node = SimpleChessArm()
    
    print("=" * 60)
    print("🤖 SIMPLE CHESS ARM CONTROLLER STARTED")
    print("This will move the robot arm when chess moves are made!")
    print("Try: ros2 run chess_robot_simulation chess_move_client e2e4")
    print("=" * 60)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutting down...')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 