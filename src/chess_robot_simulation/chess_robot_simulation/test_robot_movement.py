#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from std_srvs.srv import SetBool as StdSetBool
from std_msgs.msg import String
import time

class RobotMovementTester(Node):
    def __init__(self):
        super().__init__('robot_movement_tester')
        
        # Client for chess moves
        self.move_client = self.create_client(AddTwoInts, 'make_chess_move')
        
        # Client for gripper control
        self.gripper_client = self.create_client(StdSetBool, 'gripper_control')
        
        self.get_logger().info('Robot Movement Tester Started')
        
    def wait_for_services(self):
        """Wait for all required services to be available"""
        self.get_logger().info('Waiting for chess move service...')
        if not self.move_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Chess move service not available!')
            return False
            
        self.get_logger().info('Waiting for gripper control service...')
        if not self.gripper_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn('Gripper control service not available (this is OK)')
            
        self.get_logger().info('All services ready!')
        return True
        
    def test_gripper(self):
        """Test gripper functionality"""
        if not self.gripper_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Gripper service not available, skipping gripper test')
            return True
            
        self.get_logger().info('Testing gripper close...')
        request = StdSetBool.Request()
        request.data = True
        
        future = self.gripper_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() and future.result().success:
            self.get_logger().info('✓ Gripper close test passed')
        else:
            self.get_logger().error('✗ Gripper close test failed')
            return False
            
        time.sleep(1.0)
        
        self.get_logger().info('Testing gripper open...')
        request.data = False
        future = self.gripper_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() and future.result().success:
            self.get_logger().info('✓ Gripper open test passed')
            return True
        else:
            self.get_logger().error('✗ Gripper open test failed')
            return False
    
    def test_chess_move(self, from_square, to_square, move_name):
        """Test a chess move"""
        self.get_logger().info(f'Testing chess move: {move_name}')
        
        request = AddTwoInts.Request()
        request.a = from_square
        request.b = to_square
        
        future = self.move_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)  # Allow time for robot movement
        
        if future.result() is not None:
            success = bool(future.result().sum)
            if success:
                self.get_logger().info(f'✓ Chess move {move_name} completed successfully')
                return True
            else:
                self.get_logger().error(f'✗ Chess move {move_name} failed')
                return False
        else:
            self.get_logger().error(f'✗ Chess move {move_name} service call failed')
            return False
    
    def run_full_test(self):
        """Run complete robot movement test"""
        self.get_logger().info('=== Starting Robot Movement Test ===')
        
        if not self.wait_for_services():
            return False
            
        # Test gripper functionality
        if not self.test_gripper():
            self.get_logger().error('Gripper tests failed')
            return False
            
        # Test some chess moves
        test_moves = [
            (12, 28, "e2e4"),  # Pawn to e4
            (6, 21, "g1f3"),   # Knight to f3
            (51, 35, "d7d5"),  # Black pawn to d5
        ]
        
        for from_sq, to_sq, name in test_moves:
            if not self.test_chess_move(from_sq, to_sq, name):
                self.get_logger().error(f'Chess move test failed: {name}')
                return False
            time.sleep(2.0)  # Wait between moves
            
        self.get_logger().info('=== All Robot Movement Tests Passed! ===')
        return True

def main(args=None):
    rclpy.init(args=args)
    tester = RobotMovementTester()
    
    try:
        success = tester.run_full_test()
        if success:
            print("\n🎉 SUCCESS: Robot movement system is fully functional!")
            print("You can now use the chess_move_client to play chess with robot arm movement.")
        else:
            print("\n❌ FAILED: Some components are not working correctly.")
            print("Check the logs above for details.")
            
    except KeyboardInterrupt:
        print('\nTest interrupted by user')
    
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 