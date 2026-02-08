#!/usr/bin/env python3
"""
State Machine Action Client Node
Coordinates the entire charger plugging mission using ROS2 Actions.

Sequence:
1. NAV_TO_STATION - Navigate to charging station
2. DETECT_HANDLE - Detect charger handle
3. MANIPULATE_GRASP - Grasp the handle
4. NAV_TO_CAR - Navigate to car
5. DETECT_PORT - Detect charging port
6. MANIPULATE_INSERT - Insert plug into port
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from enum import Enum

from charging_interfaces.action import Navigate, Manipulate, Detect


class MissionState(Enum):
    """Enum for mission states."""
    IDLE = 0
    NAV_TO_STATION = 1
    DETECT_HANDLE = 2
    MANIPULATE_GRASP = 3
    NAV_TO_CAR = 4
    DETECT_PORT = 5
    MANIPULATE_INSERT = 6
    COMPLETED = 7
    ERROR = 8


class StateMachineActionClient(Node):
    """State machine node acting as an Action Client for mission coordination."""

    def __init__(self):
        super().__init__('state_machine_action_client')
        
        # Declare parameters
        self.declare_parameter('station_position_x', 2.0)
        self.declare_parameter('station_position_y', 0.0)
        self.declare_parameter('station_position_z', 0.0)
        self.declare_parameter('car_position_x', 5.0)
        self.declare_parameter('car_position_y', 2.0)
        self.declare_parameter('car_position_z', 0.0)
        
        # Get parameters
        self.station_x = self.get_parameter('station_position_x').value
        self.station_y = self.get_parameter('station_position_y').value
        self.station_z = self.get_parameter('station_position_z').value
        self.car_x = self.get_parameter('car_position_x').value
        self.car_y = self.get_parameter('car_position_y').value
        self.car_z = self.get_parameter('car_position_z').value
        
        # State
        self.current_state = MissionState.IDLE
        
        # Action Clients
        self._navigate_client = ActionClient(self, Navigate, 'navigate')
        self._manipulate_client = ActionClient(self, Manipulate, 'manipulate')
        self._detect_client = ActionClient(self, Detect, 'detect')
        
        # Publishers
        self.state_pub = self.create_publisher(
            String,
            '/mission_control/state',
            10
        )
        
        # Data
        self.detected_handle_pose = None
        self.detected_port_pose = None
        
        self.get_logger().info('State Machine Action Client initialized')
        self.get_logger().info('Waiting for action servers...')
        
        # Wait for action servers
        self._navigate_client.wait_for_server()
        self._manipulate_client.wait_for_server()
        self._detect_client.wait_for_server()
        
        self.get_logger().info('All action servers available!')
        
        # Auto-start mission
        self.create_timer(1.0, self.start_mission_once)

    def start_mission_once(self):
        """Start mission once after initialization."""
        if self.current_state == MissionState.IDLE:
            self.get_logger().info('Starting charger plugging mission')
            self.transition_to_state(MissionState.NAV_TO_STATION)
            self.execute_nav_to_station()

    def transition_to_state(self, new_state):
        """Transition to a new state."""
        self.current_state = new_state
        self.get_logger().info(f'State: {self.current_state.name}')
        
        # Publish state change
        msg = String()
        msg.data = self.current_state.name
        self.state_pub.publish(msg)

    def execute_nav_to_station(self):
        """Execute navigation to charging station."""
        self.get_logger().info('Sending goal: Navigate to station')
        
        goal_msg = Navigate.Goal()
        goal_msg.target_pose.position.x = self.station_x
        goal_msg.target_pose.position.y = self.station_y
        goal_msg.target_pose.position.z = self.station_z
        goal_msg.target_pose.orientation.w = 1.0
        
        self._send_goal_future = self._navigate_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        self._send_goal_future.add_done_callback(self.nav_to_station_response_callback)

    def nav_feedback_callback(self, feedback_msg):
        """Handle navigation feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Navigation feedback: distance_remaining={feedback.distance_remaining}')

    def nav_to_station_response_callback(self, future):
        """Handle navigation goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            self.transition_to_state(MissionState.ERROR)
            return
        
        self.get_logger().info('Navigation goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.nav_to_station_result_callback)

    def nav_to_station_result_callback(self, future):
        """Handle navigation result."""
        result = future.result().result
        if result.success:
            self.get_logger().info(f'Navigation succeeded: {result.message}')
            self.transition_to_state(MissionState.DETECT_HANDLE)
            self.execute_detect_handle()
        else:
            self.get_logger().error('Navigation failed')
            self.transition_to_state(MissionState.ERROR)

    def execute_detect_handle(self):
        """Execute detection of charger handle."""
        self.get_logger().info('Sending goal: Detect charger handle')
        
        goal_msg = Detect.Goal()
        goal_msg.object_name = 'charger_handle'
        
        self._send_goal_future = self._detect_client.send_goal_async(
            goal_msg,
            feedback_callback=self.detect_feedback_callback
        )
        self._send_goal_future.add_done_callback(self.detect_handle_response_callback)

    def detect_feedback_callback(self, feedback_msg):
        """Handle detection feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Detection feedback: confidence={feedback.confidence}%')

    def detect_handle_response_callback(self, future):
        """Handle detection goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Detection goal rejected')
            self.transition_to_state(MissionState.ERROR)
            return
        
        self.get_logger().info('Detection goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.detect_handle_result_callback)

    def detect_handle_result_callback(self, future):
        """Handle detection result."""
        result = future.result().result
        if result.success:
            self.detected_handle_pose = result.detected_pose
            self.get_logger().info(f'Handle detected at [{result.detected_pose.position.x}, '
                                 f'{result.detected_pose.position.y}, {result.detected_pose.position.z}]')
            self.transition_to_state(MissionState.MANIPULATE_GRASP)
            self.execute_manipulate_grasp()
        else:
            self.get_logger().error('Handle detection failed')
            self.transition_to_state(MissionState.ERROR)

    def execute_manipulate_grasp(self):
        """Execute grasping of charger handle."""
        self.get_logger().info('Sending goal: Grasp charger handle')
        
        goal_msg = Manipulate.Goal()
        goal_msg.task_type = 'grasp_handle'
        
        self._send_goal_future = self._manipulate_client.send_goal_async(
            goal_msg,
            feedback_callback=self.manipulate_feedback_callback
        )
        self._send_goal_future.add_done_callback(self.manipulate_grasp_response_callback)

    def manipulate_feedback_callback(self, feedback_msg):
        """Handle manipulation feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Manipulation feedback: {feedback.status}')

    def manipulate_grasp_response_callback(self, future):
        """Handle manipulation goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Manipulation goal rejected')
            self.transition_to_state(MissionState.ERROR)
            return
        
        self.get_logger().info('Manipulation goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.manipulate_grasp_result_callback)

    def manipulate_grasp_result_callback(self, future):
        """Handle manipulation result."""
        result = future.result().result
        if result.success:
            self.get_logger().info('Grasp successful')
            self.transition_to_state(MissionState.NAV_TO_CAR)
            self.execute_nav_to_car()
        else:
            self.get_logger().error('Grasp failed')
            self.transition_to_state(MissionState.ERROR)

    def execute_nav_to_car(self):
        """Execute navigation to car."""
        self.get_logger().info('Sending goal: Navigate to car')
        
        goal_msg = Navigate.Goal()
        goal_msg.target_pose.position.x = self.car_x
        goal_msg.target_pose.position.y = self.car_y
        goal_msg.target_pose.position.z = self.car_z
        goal_msg.target_pose.orientation.w = 1.0
        
        self._send_goal_future = self._navigate_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        self._send_goal_future.add_done_callback(self.nav_to_car_response_callback)

    def nav_to_car_response_callback(self, future):
        """Handle navigation goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            self.transition_to_state(MissionState.ERROR)
            return
        
        self.get_logger().info('Navigation goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.nav_to_car_result_callback)

    def nav_to_car_result_callback(self, future):
        """Handle navigation result."""
        result = future.result().result
        if result.success:
            self.get_logger().info(f'Navigation to car succeeded: {result.message}')
            self.transition_to_state(MissionState.DETECT_PORT)
            self.execute_detect_port()
        else:
            self.get_logger().error('Navigation to car failed')
            self.transition_to_state(MissionState.ERROR)

    def execute_detect_port(self):
        """Execute detection of charging port."""
        self.get_logger().info('Sending goal: Detect charging port')
        
        goal_msg = Detect.Goal()
        goal_msg.object_name = 'charging_port'
        
        self._send_goal_future = self._detect_client.send_goal_async(
            goal_msg,
            feedback_callback=self.detect_feedback_callback
        )
        self._send_goal_future.add_done_callback(self.detect_port_response_callback)

    def detect_port_response_callback(self, future):
        """Handle detection goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Detection goal rejected')
            self.transition_to_state(MissionState.ERROR)
            return
        
        self.get_logger().info('Detection goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.detect_port_result_callback)

    def detect_port_result_callback(self, future):
        """Handle detection result."""
        result = future.result().result
        if result.success:
            self.detected_port_pose = result.detected_pose
            self.get_logger().info(f'Port detected at [{result.detected_pose.position.x}, '
                                 f'{result.detected_pose.position.y}, {result.detected_pose.position.z}]')
            self.transition_to_state(MissionState.MANIPULATE_INSERT)
            self.execute_manipulate_insert()
        else:
            self.get_logger().error('Port detection failed')
            self.transition_to_state(MissionState.ERROR)

    def execute_manipulate_insert(self):
        """Execute insertion of charger plug."""
        self.get_logger().info('Sending goal: Insert charger plug')
        
        goal_msg = Manipulate.Goal()
        goal_msg.task_type = 'insert_plug'
        
        self._send_goal_future = self._manipulate_client.send_goal_async(
            goal_msg,
            feedback_callback=self.manipulate_feedback_callback
        )
        self._send_goal_future.add_done_callback(self.manipulate_insert_response_callback)

    def manipulate_insert_response_callback(self, future):
        """Handle manipulation goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Manipulation goal rejected')
            self.transition_to_state(MissionState.ERROR)
            return
        
        self.get_logger().info('Manipulation goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.manipulate_insert_result_callback)

    def manipulate_insert_result_callback(self, future):
        """Handle manipulation result."""
        result = future.result().result
        if result.success:
            self.get_logger().info('Insertion successful!')
            self.transition_to_state(MissionState.COMPLETED)
            self.get_logger().info('MISSION COMPLETED SUCCESSFULLY!')
        else:
            self.get_logger().error('Insertion failed')
            self.transition_to_state(MissionState.ERROR)


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineActionClient()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
