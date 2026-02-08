#!/usr/bin/env python3
"""
Manipulation Action Server
Mock implementation that simulates manipulation tasks for testing.
"""
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from charging_interfaces.action import Manipulate


class ManipulationActionServer(Node):
    """Mock action server for manipulation tasks."""

    def __init__(self):
        super().__init__('manipulation_action_server')
        
        self._action_server = ActionServer(
            self,
            Manipulate,
            'manipulate',
            self.execute_callback
        )
        
        self.get_logger().info('Manipulation Action Server initialized')

    def execute_callback(self, goal_handle):
        """Execute the manipulation action."""
        self.get_logger().info(f'Received manipulation goal: {goal_handle.request.task_type}')
        
        # Simulate manipulation with feedback
        feedback_msg = Manipulate.Feedback()
        statuses = ['Preparing', 'Moving arm', 'Adjusting position', 'Executing task', 'Completing']
        
        for status in statuses:
            feedback_msg.status = status
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Status: {status}')
            time.sleep(0.4)
        
        # Mark goal as succeeded
        goal_handle.succeed()
        
        result = Manipulate.Result()
        result.success = True
        
        self.get_logger().info(f'Manipulation task "{goal_handle.request.task_type}" succeeded')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ManipulationActionServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
