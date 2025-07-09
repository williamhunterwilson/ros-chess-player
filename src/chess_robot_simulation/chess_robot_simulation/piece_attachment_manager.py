#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetLinkState, GetLinkState, SetModelState, GetModelState
from gazebo_msgs.msg import LinkState, ModelState
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, TransformStamped
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster
import numpy as np

class PieceAttachmentManager(Node):
    """Manages attachment/detachment of chess pieces to gripper"""
    
    def __init__(self):
        super().__init__('piece_attachment_manager')
        
        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Service clients for Gazebo (if using Gazebo)
        self.set_model_state_client = self.create_client(SetModelState, '/gazebo/set_model_state')
        self.get_model_state_client = self.create_client(GetModelState, '/gazebo/get_model_state')
        
        # For non-Gazebo simulations, we'll track gripper pose from TF
        self.gripper_pose = Pose()
        self.gripper_frame = 'gripper_link'  # Adjust based on your URDF
        self.world_frame = 'world'
        
        # Subscribers
        self.attach_sub = self.create_subscription(
            String, '/attach_piece', self.attach_piece_callback, 10)
        self.detach_sub = self.create_subscription(
            Bool, '/detach_piece', self.detach_piece_callback, 10)
        
        # State
        self.attached_piece = None
        self.piece_offset = Point()
        self.piece_offset.z = -0.04  # Default offset below gripper
        
        # Timer to update attached piece position
        self.timer = self.create_timer(0.01, self.update_attached_piece)
        
        self.get_logger().info('Piece Attachment Manager initialized')
    
    def attach_piece_callback(self, msg):
        """Attach specified piece to gripper"""
        piece_name = msg.data
        
        # Store the piece name
        self.attached_piece = piece_name
        
        # Calculate offset from gripper to piece at pickup
        try:
            # Get current gripper transform
            gripper_transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.gripper_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Get current piece transform
            piece_transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                piece_name,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Calculate offset
            self.piece_offset.x = piece_transform.transform.translation.x - gripper_transform.transform.translation.x
            self.piece_offset.y = piece_transform.transform.translation.y - gripper_transform.transform.translation.y
            self.piece_offset.z = piece_transform.transform.translation.z - gripper_transform.transform.translation.z
            
        except Exception as e:
            self.get_logger().warn(f"Could not calculate exact offset, using default: {e}")
            self.piece_offset.x = 0.0
            self.piece_offset.y = 0.0
            self.piece_offset.z = -0.04
        
        self.get_logger().info(f"Attached piece: {self.attached_piece} with offset ({self.piece_offset.x:.3f}, {self.piece_offset.y:.3f}, {self.piece_offset.z:.3f})")
    
    def detach_piece_callback(self, msg):
        """Detach current piece from gripper"""
        if self.attached_piece:
            self.get_logger().info(f"Detached piece: {self.attached_piece}")
            self.attached_piece = None
    
    def update_attached_piece(self):
        """Update position of attached piece to follow gripper"""
        if not self.attached_piece:
            return
        
        try:
            # Get current gripper transform from TF
            gripper_transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.gripper_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Create new pose for the piece
            piece_pose = Pose()
            piece_pose.position.x = gripper_transform.transform.translation.x + self.piece_offset.x
            piece_pose.position.y = gripper_transform.transform.translation.y + self.piece_offset.y
            piece_pose.position.z = gripper_transform.transform.translation.z + self.piece_offset.z
            piece_pose.orientation = gripper_transform.transform.rotation
            
            # If using Gazebo, update via service
            if self.set_model_state_client.service_is_ready():
                self.update_piece_in_gazebo(self.attached_piece, piece_pose)
            
            # Also publish as TF transform (handled by main node)
            
        except Exception as e:
            # Gripper transform not available yet
            pass
    
    def update_piece_in_gazebo(self, piece_name, pose):
        """Update piece position in Gazebo"""
        model_state = ModelState()
        model_state.model_name = piece_name
        model_state.pose = pose
        model_state.twist = Twist()  # Zero velocity
        model_state.reference_frame = self.world_frame
        
        request = SetModelState.Request()
        request.model_state = model_state
        
        # Asynchronous call
        future = self.set_model_state_client.call_async(request)
        # Don't wait for response to avoid blocking

class PieceAttachmentManagerAlternative(Node):
    """Alternative implementation using direct transform broadcasting"""
    
    def __init__(self):
        super().__init__('piece_attachment_manager_alt')
        
        # Transform broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.attach_sub = self.create_subscription(
            String, '/attach_piece', self.attach_piece_callback, 10)
        self.detach_sub = self.create_subscription(
            Bool, '/detach_piece', self.detach_piece_callback, 10)
        
        # State
        self.attached_piece = None
        self.piece_to_gripper_transform = None
        
        # Timer for publishing transforms
        self.timer = self.create_timer(0.01, self.publish_attached_piece_transform)
        
        self.get_logger().info('Alternative Piece Attachment Manager initialized')
    
    def attach_piece_callback(self, msg):
        """Attach piece to gripper by creating a fixed transform"""
        piece_name = msg.data
        self.attached_piece = piece_name
        
        # Create a static transform from gripper to piece
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'gripper_link'
        transform.child_frame_id = f'{piece_name}_attached'
        
        # Piece hangs below gripper
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = -0.04
        
        # No rotation
        transform.transform.rotation.w = 1.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        
        self.piece_to_gripper_transform = transform
        
        self.get_logger().info(f"Attached piece: {piece_name}")
    
    def detach_piece_callback(self, msg):
        """Detach piece from gripper"""
        if self.attached_piece:
            self.get_logger().info(f"Detached piece: {self.attached_piece}")
            self.attached_piece = None
            self.piece_to_gripper_transform = None
    
    def publish_attached_piece_transform(self):
        """Continuously publish transform for attached piece"""
        if self.attached_piece and self.piece_to_gripper_transform:
            # Update timestamp
            self.piece_to_gripper_transform.header.stamp = self.get_clock().now().to_msg()
            
            # Publish transform
            self.tf_broadcaster.sendTransform(self.piece_to_gripper_transform)
            
            # Also publish piece position in world frame
            try:
                # Get attached piece position in world frame
                world_to_attached = self.tf_buffer.lookup_transform(
                    'world',
                    f'{self.attached_piece}_attached',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                
                # Publish this as the actual piece location
                piece_transform = TransformStamped()
                piece_transform.header = world_to_attached.header
                piece_transform.child_frame_id = self.attached_piece
                piece_transform.transform = world_to_attached.transform
                
                self.tf_broadcaster.sendTransform(piece_transform)
                
            except Exception:
                # Transform not ready yet
                pass

def main(args=None):
    rclpy.init(args=args)
    
    # Choose which implementation to use
    # Use the first one if you have Gazebo integration
    # Use the alternative for pure TF-based attachment
    
    use_alternative = True  # Set based on your setup
    
    if use_alternative:
        node = PieceAttachmentManagerAlternative()
    else:
        node = PieceAttachmentManager()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()