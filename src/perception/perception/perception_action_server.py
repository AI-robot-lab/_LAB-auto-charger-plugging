#!/usr/bin/env python3
"""
Perception Action Server
Mock implementation that simulates object detection tasks for testing.
"""
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from charging_interfaces.action import Detect


class PerceptionActionServer(Node):
    """Mock action server for perception tasks."""

    def __init__(self):
        super().__init__('perception_action_server')
        
        self._action_server = ActionServer(
            self,
            Detect,
            'detect',
            self.execute_callback
        )
        
        self.get_logger().info('Perception Action Server initialized')

    def execute_callback(self, goal_handle):
        """Execute the detection action."""
        self.get_logger().info(f'Received detection goal for object: {goal_handle.request.object_name}')
        
        # Simulate detection with feedback
        feedback_msg = Detect.Feedback()
        for i in range(10):
            feedback_msg.confidence = (i + 1) * 10
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Detection confidence: {feedback_msg.confidence}%')
            time.sleep(0.2)
        
        # Mark goal as succeeded
        goal_handle.succeed()
        
        result = Detect.Result()
        result.success = True
        # Mock detected pose
        result.detected_pose.position.x = 1.0
        result.detected_pose.position.y = 0.5
        result.detected_pose.position.z = 0.8
        result.detected_pose.orientation.w = 1.0
        
        self.get_logger().info(f'Object "{goal_handle.request.object_name}" detected successfully')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionActionServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
