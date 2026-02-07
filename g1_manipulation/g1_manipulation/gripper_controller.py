#!/usr/bin/env python3
"""
Gripper Controller Node
Controls the Unitree G1 gripper for grasping operations.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import JointState


class GripperController(Node):
    """Node for controlling the Unitree G1 gripper."""

    def __init__(self):
        super().__init__('gripper_controller')
        
        # Declare parameters
        self.declare_parameter('gripper_name', 'right_gripper')
        self.declare_parameter('max_force', 50.0)
        self.declare_parameter('grasp_threshold', 0.02)
        
        # Get parameters
        self.gripper_name = self.get_parameter('gripper_name').value
        self.max_force = self.get_parameter('max_force').value
        self.grasp_threshold = self.get_parameter('grasp_threshold').value
        
        # State
        self.current_position = 0.0
        self.is_grasping = False
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.grasp_command_sub = self.create_subscription(
            Bool,
            f'/manipulation/{self.gripper_name}/grasp',
            self.grasp_command_callback,
            10
        )
        
        # Publishers
        self.gripper_position_pub = self.create_publisher(
            Float64,
            f'/{self.gripper_name}/position_command',
            10
        )
        
        self.grasp_status_pub = self.create_publisher(
            Bool,
            f'/manipulation/{self.gripper_name}/grasp_status',
            10
        )
        
        # Timer for status updates
        self.status_timer = self.create_timer(0.1, self.publish_status)
        
        self.get_logger().info(f'Gripper Controller Node initialized for {self.gripper_name}')

    def joint_state_callback(self, msg):
        """Update current gripper joint state."""
        # TODO: Extract gripper joint position from joint states
        pass

    def grasp_command_callback(self, msg):
        """Execute grasp or release command."""
        if msg.data:
            self.close_gripper()
        else:
            self.open_gripper()

    def close_gripper(self):
        """Close the gripper to grasp an object."""
        self.get_logger().info('Closing gripper to grasp')
        
        # TODO: Implement actual gripper closing
        # This is a placeholder for the control logic
        # You would typically:
        # - Send position/force commands to gripper
        # - Monitor force/current feedback
        # - Detect successful grasp
        # - Interface with Unitree SDK
        
        self.is_grasping = True

    def open_gripper(self):
        """Open the gripper to release an object."""
        self.get_logger().info('Opening gripper to release')
        
        # TODO: Implement actual gripper opening
        self.is_grasping = False

    def publish_status(self):
        """Publish current grasp status."""
        msg = Bool()
        msg.data = self.is_grasping
        self.grasp_status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GripperController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
