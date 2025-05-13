import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint
from builtin_interfaces.msg import Duration
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import chess
import numpy as np

SQUARE_SIZE = 0.05
BOARD_SIZE = 8 * SQUARE_SIZE
BOARD_Z = 0.01
ROBOT_BASE_X = -0.3
ROBOT_BASE_Y = BOARD_SIZE / 2
ROBOT_BASE_Z = 0.0

WHITE = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
BLACK = ColorRGBA(r=0.2, g=0.2, b=0.2, a=1.0)
LIGHT_SQUARE = ColorRGBA(r=0.9, g=0.8, b=0.6, a=1.0)
DARK_SQUARE = ColorRGBA(r=0.5, g=0.4, b=0.3, a=1.0)

class ChessRobotNode(Node):
    def __init__(self):
        super().__init__('chess_robot_node')

        self.board_publisher = self.create_publisher(Marker, 'chess_board_marker', 10)
        self.pieces_publisher = self.create_publisher(MarkerArray, 'chess_pieces_markers', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self._action_client = ActionClient(self, MoveGroup, 'move_action')

        self.board = chess.Board()
        self.board_marker = None
        self.piece_markers = MarkerArray()

        self.create_timer(0.1, self.publish_board)
        self.create_timer(0.1, self.publish_pieces)
        self.create_timer(0.1, self.publish_robot_base_tf)

        self.create_board_marker()
        self.get_logger().info('Chess Robot Node Started')

    def publish_robot_base_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'robot_base'
        t.transform.translation.x = ROBOT_BASE_X
        t.transform.translation.y = ROBOT_BASE_Y
        t.transform.translation.z = ROBOT_BASE_Z
        t.transform.rotation.w = 1.0  # No rotation for simplicity
        self.tf_broadcaster.sendTransform(t)

    def create_board_marker(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.ns = "chess_board"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = BOARD_SIZE / 2
        marker.pose.position.y = BOARD_SIZE / 2
        marker.pose.position.z = BOARD_Z / 2
        marker.pose.orientation.w = 1.0
        marker.scale.x = BOARD_SIZE
        marker.scale.y = BOARD_SIZE
        marker.scale.z = BOARD_Z
        marker.color.r = 0.8
        marker.color.g = 0.6
        marker.color.b = 0.4
        marker.color.a = 1.0
        self.board_marker = marker

    def publish_board(self):
        if self.board_marker:
            self.board_publisher.publish(self.board_marker)

    def publish_pieces(self):
        self.pieces_publisher.publish(self.piece_markers)

    def get_square_position(self, file, rank):
        return {
            'x': file * SQUARE_SIZE + SQUARE_SIZE / 2,
            'y': rank * SQUARE_SIZE + SQUARE_SIZE / 2,
            'z': BOARD_Z + 0.1
        }

    def send_move_goal(self, position):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup action server not available.')
            return

        goal_msg = MoveGroup.Goal()
        goal_msg.request = MotionPlanRequest()
        goal_msg.request.group_name = "ur5_arm"
        goal_msg.allowed_planning_time = 5.0

        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = position['x']
        pose.pose.position.y = position['y']
        pose.pose.position.z = position['z']
        pose.pose.orientation.w = 1.0

        constraint = Constraints()
        constraint.name = "pose_constraint"

        pc = PositionConstraint()
        pc.weight = 1.0
        pc.target_point_offset = pose.pose.position
        constraint.position_constraints.append(pc)

        goal_msg.request.goal_constraints.append(constraint)

        self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info("Feedback received from MoveGroup")

def main(args=None):
    rclpy.init(args=args)
    node = ChessRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
