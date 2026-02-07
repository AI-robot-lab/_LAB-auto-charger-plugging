#!/usr/bin/env python3
"""
Walking Controller Node
Controls the walking gait and movement of the Unitree G1 robot.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry


class WalkingController(Node):
    """Node for controlling Unitree G1 walking and movement."""

    def __init__(self):
        super().__init__('walking_controller')
        
        # Declare parameters
        self.declare_parameter('max_linear_velocity', 0.5)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('position_tolerance', 0.05)
        
        # Get parameters
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        
        # State
        self.current_pose = None
        self.target_pose = None
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/navigation/goal',
            self.goal_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Walking Controller Node initialized')

    def odom_callback(self, msg):
        """Update current pose from odometry."""
        self.current_pose = msg.pose.pose

    def goal_callback(self, msg):
        """Receive new navigation goal."""
        self.target_pose = msg.pose
        self.get_logger().info(f'New goal received at position: '
                              f'({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

    def control_loop(self):
        """Main control loop for walking."""
        if self.current_pose is None or self.target_pose is None:
            return
        
        # TODO: Implement actual walking control using Unitree SDK
        # This is a placeholder for the control logic
        # You would typically:
        # - Calculate distance and angle to target
        # - Generate appropriate velocity commands
        # - Interface with Unitree SDK for low-level control
        # - Handle obstacle avoidance
        # - Implement smooth motion profiles
        
        pass


def main(args=None):
    rclpy.init(args=args)
    node = WalkingController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
