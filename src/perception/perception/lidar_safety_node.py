#!/usr/bin/env python3
"""
LIDAR Safety Node
Monitors LIDAR data for obstacles and triggers emergency stop if needed.
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class LidarSafetyNode(Node):
    """Node for monitoring LIDAR data and ensuring safe navigation."""

    def __init__(self):
        super().__init__('lidar_safety_node')
        
        # Declare parameters
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('safety_distance', 0.5)  # meters
        self.declare_parameter('front_arc_degrees', 60.0)  # degrees
        self.declare_parameter('check_rate', 10.0)  # Hz
        
        # Get parameters
        scan_topic = self.get_parameter('scan_topic').value
        self.safety_distance = self.get_parameter('safety_distance').value
        self.front_arc_degrees = self.get_parameter('front_arc_degrees').value
        check_rate = self.get_parameter('check_rate').value
        
        # Convert front arc to radians
        self.front_arc_radians = math.radians(self.front_arc_degrees)
        
        # State
        self.latest_scan = None
        self.obstacle_detected = False
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10
        )
        
        # Publishers
        self.emergency_stop_pub = self.create_publisher(
            Bool,
            '/safety/emergency_stop',
            10
        )
        
        # Timer for periodic safety checks
        check_period = 1.0 / check_rate
        self.safety_timer = self.create_timer(check_period, self.check_safety)
        
        self.get_logger().info(f'LIDAR Safety Node initialized')
        self.get_logger().info(f'Safety distance: {self.safety_distance}m, Front arc: {self.front_arc_degrees}°')

    def scan_callback(self, msg):
        """Store latest LIDAR scan."""
        self.latest_scan = msg

    def check_safety(self):
        """Check LIDAR data for obstacles in the safety zone."""
        if self.latest_scan is None:
            return
        
        scan = self.latest_scan
        obstacle_found = False
        min_distance = float('inf')
        
        # Calculate the indices for the front arc
        # LaserScan ranges are typically from right to left (CW from -angle_max to +angle_max)
        # We want to check the front arc centered at 0 degrees
        half_arc = self.front_arc_radians / 2.0
        
        for i, distance in enumerate(scan.ranges):
            # Skip invalid readings
            if math.isinf(distance) or math.isnan(distance):
                continue
            
            # Calculate angle for this reading
            angle = scan.angle_min + (i * scan.angle_increment)
            
            # Check if this reading is within the front arc
            if abs(angle) <= half_arc:
                # Check if obstacle is within safety distance
                if distance < self.safety_distance:
                    obstacle_found = True
                    min_distance = min(min_distance, distance)
        
        # Publish emergency stop signal if obstacle detected
        msg = Bool()
        msg.data = obstacle_found
        self.emergency_stop_pub.publish(msg)
        
        # Log state changes
        if obstacle_found and not self.obstacle_detected:
            self.get_logger().warn(f'OBSTACLE DETECTED! Minimum distance: {min_distance:.2f}m')
        elif not obstacle_found and self.obstacle_detected:
            self.get_logger().info('Path clear, obstacle cleared')
        
        self.obstacle_detected = obstacle_found


def main(args=None):
    rclpy.init(args=args)
    node = LidarSafetyNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
