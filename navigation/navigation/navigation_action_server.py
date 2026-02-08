#!/usr/bin/env python3
"""
Navigation Action Server
Mock implementation that simulates navigation tasks for testing.
"""
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from charging_interfaces.action import Navigate


class NavigationActionServer(Node):
    """Mock action server for navigation tasks."""

    def __init__(self):
        super().__init__('navigation_action_server')
        
        self._action_server = ActionServer(
            self,
            Navigate,
            'navigate',
            self.execute_callback
        )
        
        self.get_logger().info('Navigation Action Server initialized')

    def execute_callback(self, goal_handle):
        """Execute the navigation action."""
        self.get_logger().info(f'Received navigation goal to pose: '
                             f'[{goal_handle.request.target_pose.position.x}, '
                             f'{goal_handle.request.target_pose.position.y}, '
                             f'{goal_handle.request.target_pose.position.z}]')
        
        # Simulate navigation with feedback
        feedback_msg = Navigate.Feedback()
        for i in range(10):
            feedback_msg.distance_remaining = 10.0 - i
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Distance remaining: {feedback_msg.distance_remaining}')
            time.sleep(0.2)
        
        # Mark goal as succeeded
        goal_handle.succeed()
        
        result = Navigate.Result()
        result.success = True
        result.message = 'Navigation completed successfully'
        
        self.get_logger().info('Navigation goal succeeded')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = NavigationActionServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
