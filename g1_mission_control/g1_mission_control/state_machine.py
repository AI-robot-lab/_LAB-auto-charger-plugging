#!/usr/bin/env python3
"""
State Machine Node
Coordinates the entire charger plugging mission using a finite state machine.

States:
1. IDLE: Wait for mission start
2. NAV_TO_STATION: Navigate to the charging station
3. ALIGN_WITH_CHARGER: Align with the charger handle
4. GRASP_HANDLE: Grasp the charger handle
5. NAV_TO_CAR: Navigate to the car with the charger
6. ALIGN_WITH_PORT: Align with the car charging port
7. INSERT_PLUG: Insert the charger plug into the car port
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from enum import Enum


class MissionState(Enum):
    """Enum for mission states."""
    IDLE = 0
    NAV_TO_STATION = 1
    ALIGN_WITH_CHARGER = 2
    GRASP_HANDLE = 3
    NAV_TO_CAR = 4
    ALIGN_WITH_PORT = 5
    INSERT_PLUG = 6
    COMPLETED = 7
    ERROR = 8


class StateMachine(Node):
    """State machine node for coordinating the charger plugging mission."""

    def __init__(self):
        super().__init__('state_machine')
        
        # Declare parameters
        self.declare_parameter('auto_start', False)
        self.declare_parameter('station_location_x', 0.0)
        self.declare_parameter('station_location_y', 0.0)
        self.declare_parameter('car_position_x', 5.0)
        self.declare_parameter('car_position_y', 0.0)
        self.declare_parameter('charger_handle_height', 0.9)
        self.declare_parameter('car_port_height', 0.8)
        
        # Get parameters
        self.auto_start = self.get_parameter('auto_start').value
        self.station_x = self.get_parameter('station_location_x').value
        self.station_y = self.get_parameter('station_location_y').value
        self.car_x = self.get_parameter('car_position_x').value
        self.car_y = self.get_parameter('car_position_y').value
        self.charger_handle_height = self.get_parameter('charger_handle_height').value
        self.car_port_height = self.get_parameter('car_port_height').value
        
        # State
        self.current_state = MissionState.IDLE
        self.previous_state = None
        
        # Subscribers
        self.start_sub = self.create_subscription(
            Bool,
            '/mission_control/start',
            self.start_callback,
            10
        )
        
        self.charger_detected_sub = self.create_subscription(
            PoseStamped,
            '/perception/charger_pose',
            self.charger_detected_callback,
            10
        )
        
        self.port_detected_sub = self.create_subscription(
            PoseStamped,
            '/perception/port_pose',
            self.port_detected_callback,
            10
        )
        
        self.navigation_status_sub = self.create_subscription(
            String,
            '/navigation/status',
            self.navigation_status_callback,
            10
        )
        
        self.grasp_status_sub = self.create_subscription(
            Bool,
            '/manipulation/right_gripper/grasp_status',
            self.grasp_status_callback,
            10
        )
        
        # Publishers
        self.state_pub = self.create_publisher(
            String,
            '/mission_control/state',
            10
        )
        
        self.navigation_goal_pub = self.create_publisher(
            PoseStamped,
            '/navigation/goal',
            10
        )
        
        self.arm_target_pub = self.create_publisher(
            PoseStamped,
            '/manipulation/right_arm/target_pose',
            10
        )
        
        self.gripper_command_pub = self.create_publisher(
            Bool,
            '/manipulation/right_gripper/grasp',
            10
        )
        
        # Timer for state machine execution
        self.state_timer = self.create_timer(0.5, self.state_machine_loop)
        
        # Data
        self.charger_pose = None
        self.port_pose = None
        self.navigation_status = None
        self.is_grasping = False
        
        self.get_logger().info('State Machine Node initialized')
        
        if self.auto_start:
            self.get_logger().info('Auto-start enabled, beginning mission')
            self.start_mission()

    def start_callback(self, msg):
        """Handle manual mission start command."""
        if msg.data:
            self.start_mission()

    def start_mission(self):
        """Start the charger plugging mission."""
        if self.current_state == MissionState.IDLE:
            self.get_logger().info('Starting charger plugging mission')
            self.transition_to_state(MissionState.NAV_TO_STATION)
        else:
            self.get_logger().warning(f'Cannot start mission, current state is {self.current_state.name}')

    def charger_detected_callback(self, msg):
        """Handle charger detection."""
        self.charger_pose = msg

    def port_detected_callback(self, msg):
        """Handle port detection."""
        self.port_pose = msg

    def navigation_status_callback(self, msg):
        """Handle navigation status updates."""
        self.navigation_status = msg.data

    def grasp_status_callback(self, msg):
        """Handle grasp status updates."""
        self.is_grasping = msg.data

    def transition_to_state(self, new_state):
        """Transition to a new state."""
        self.previous_state = self.current_state
        self.current_state = new_state
        
        self.get_logger().info(f'State transition: {self.previous_state.name} -> {self.current_state.name}')
        
        # Publish state change
        msg = String()
        msg.data = self.current_state.name
        self.state_pub.publish(msg)

    def state_machine_loop(self):
        """Main state machine execution loop."""
        if self.current_state == MissionState.IDLE:
            self.handle_idle_state()
        elif self.current_state == MissionState.NAV_TO_STATION:
            self.handle_nav_to_station_state()
        elif self.current_state == MissionState.ALIGN_WITH_CHARGER:
            self.handle_align_with_charger_state()
        elif self.current_state == MissionState.GRASP_HANDLE:
            self.handle_grasp_handle_state()
        elif self.current_state == MissionState.NAV_TO_CAR:
            self.handle_nav_to_car_state()
        elif self.current_state == MissionState.ALIGN_WITH_PORT:
            self.handle_align_with_port_state()
        elif self.current_state == MissionState.INSERT_PLUG:
            self.handle_insert_plug_state()
        elif self.current_state == MissionState.COMPLETED:
            self.handle_completed_state()
        elif self.current_state == MissionState.ERROR:
            self.handle_error_state()

    def handle_idle_state(self):
        """Handle IDLE state."""
        # Wait for start command
        pass

    def handle_nav_to_station_state(self):
        """Handle NAV_TO_STATION state."""
        # Send navigation goal to charging station
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = self.station_x
        goal.pose.position.y = self.station_y
        goal.pose.orientation.w = 1.0
        
        self.navigation_goal_pub.publish(goal)
        self.get_logger().info('Navigating to charging station')
        
        # TODO: Check if station reached
        # For now, transition based on navigation status
        if self.navigation_status == 'reached':
            self.transition_to_state(MissionState.ALIGN_WITH_CHARGER)

    def handle_align_with_charger_state(self):
        """Handle ALIGN_WITH_CHARGER state."""
        if self.charger_pose is None:
            self.get_logger().warning('Waiting for charger detection')
            return

        self.get_logger().info('Aligning with charger handle')
        self.transition_to_state(MissionState.GRASP_HANDLE)

    def handle_grasp_handle_state(self):
        """Handle GRASP_HANDLE state."""
        if self.charger_pose is None:
            self.get_logger().warning('Waiting for charger detection')
            return
        
        # Send arm target to charger position
        self.arm_target_pub.publish(self.charger_pose)
        
        # Command gripper to grasp
        grasp_cmd = Bool()
        grasp_cmd.data = True
        self.gripper_command_pub.publish(grasp_cmd)
        
        self.get_logger().info('Grasping charger handle')
        
        # TODO: Verify successful grasp
        if self.is_grasping:
            self.transition_to_state(MissionState.NAV_TO_CAR)

    def handle_nav_to_car_state(self):
        """Handle NAV_TO_CAR state."""
        # Send navigation goal to car
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = self.car_x
        goal.pose.position.y = self.car_y
        goal.pose.orientation.w = 1.0
        
        self.navigation_goal_pub.publish(goal)
        self.get_logger().info('Navigating to car with charger')
        
        # TODO: Check if car reached
        if self.navigation_status == 'reached':
            self.transition_to_state(MissionState.ALIGN_WITH_PORT)

    def handle_align_with_port_state(self):
        """Handle ALIGN_WITH_PORT state."""
        if self.port_pose is None:
            self.get_logger().warning('Waiting for port detection')
            return

        self.get_logger().info('Aligning with car charging port')
        self.transition_to_state(MissionState.INSERT_PLUG)

    def handle_insert_plug_state(self):
        """Handle INSERT_PLUG state."""
        if self.port_pose is None:
            self.get_logger().warning('Waiting for port detection')
            return
        
        # Send arm target to port position for insertion
        self.arm_target_pub.publish(self.port_pose)
        
        self.get_logger().info('Inserting charger plug into port')
        
        # TODO: Verify successful insertion
        # For now, assume success after some time
        self.transition_to_state(MissionState.COMPLETED)

    def handle_completed_state(self):
        """Handle COMPLETED state."""
        self.get_logger().info('Mission completed successfully!', throttle_duration_sec=5.0)
        
        # Open gripper to release
        release_cmd = Bool()
        release_cmd.data = False
        self.gripper_command_pub.publish(release_cmd)

    def handle_error_state(self):
        """Handle ERROR state."""
        self.get_logger().error('Mission in ERROR state', throttle_duration_sec=5.0)
        # TODO: Implement error recovery logic


def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
