#!/usr/bin/env python3
"""
YOLO Detector Node
Uses YOLOv8 for object detection and depth camera for 3D localization.
GPU-accelerated detection for high accuracy.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import message_filters
import cv2

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


class YoloDetectorNode(Node):
    """Node for YOLOv8-based object detection with depth integration."""

    def __init__(self):
        super().__init__('yolo_detector_node')
        
        if not YOLO_AVAILABLE:
            self.get_logger().error('ultralytics not installed! Run: pip install ultralytics')
            raise ImportError('ultralytics package required')
        
        # Declare parameters
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('model_path', 'yolov8n.pt')  # nano model by default
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('use_gpu', True)
        self.declare_parameter('target_classes', ['charger_handle', 'car_port'])
        
        # Get parameters
        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        use_gpu = self.get_parameter('use_gpu').value
        self.target_classes = self.get_parameter('target_classes').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Camera intrinsics
        self.camera_info = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        
        # Load YOLO model
        self.get_logger().info(f'Loading YOLOv8 model: {model_path}')
        device = 'cuda:0' if use_gpu else 'cpu'
        try:
            self.model = YOLO(model_path)
            self.model.to(device)
            self.get_logger().info(f'Model loaded successfully on {device}')
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            raise
        
        # Subscribe to camera info
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            10
        )
        
        # Synchronized RGB + Depth subscribers
        rgb_sub = message_filters.Subscriber(self, Image, rgb_topic)
        depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        
        # ApproximateTimeSynchronizer for RGB and Depth
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub],
            queue_size=10,
            slop=0.1  # 100ms tolerance
        )
        self.ts.registerCallback(self.synchronized_callback)
        
        # Publishers for detected objects
        self.charger_pose_pub = self.create_publisher(
            PoseStamped,
            '/perception/charger_pose',
            10
        )
        
        self.port_pose_pub = self.create_publisher(
            PoseStamped,
            '/perception/port_pose',
            10
        )
        
        # Debug image publisher (optional)
        self.debug_image_pub = self.create_publisher(
            Image,
            '/perception/debug_image',
            10
        )
        
        self.get_logger().info('YOLO Detector Node initialized')

    def camera_info_callback(self, msg):
        """Store camera intrinsics from camera_info."""
        if self.camera_info is None:
            self.camera_info = msg
            # Extract intrinsic parameters
            # K = [fx  0 cx]
            #     [ 0 fy cy]
            #     [ 0  0  1]
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info(f'Camera intrinsics received: fx={self.fx:.1f}, fy={self.fy:.1f}')

    def deproject_pixel_to_point(self, pixel_x, pixel_y, depth_value):
        """
        Convert 2D pixel + depth to 3D point using pinhole camera model.
        
        Args:
            pixel_x: X coordinate in image (pixels)
            pixel_y: Y coordinate in image (pixels)
            depth_value: Depth at that pixel (meters)
            
        Returns:
            (x, y, z) in camera frame (meters)
        """
        if self.fx is None:
            return None
        
        # Pinhole camera model
        x = (pixel_x - self.cx) * depth_value / self.fx
        y = (pixel_y - self.cy) * depth_value / self.fy
        z = depth_value
        
        return (x, y, z)

    def synchronized_callback(self, rgb_msg, depth_msg):
        """Process synchronized RGB and Depth images."""
        try:
            # Convert images
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            
            # Convert depth to meters if needed (depends on camera)
            # RealSense typically outputs in mm, convert to meters
            if depth_image.dtype == np.uint16:
                depth_image = depth_image.astype(np.float32) / 1000.0
            
            # Run YOLO detection
            results = self.model(rgb_image, conf=self.confidence_threshold, verbose=False)
            
            # Process detections
            debug_image = rgb_image.copy()
            
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    # Get bounding box
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = box.conf[0].cpu().numpy()
                    cls = int(box.cls[0].cpu().numpy())
                    class_name = result.names[cls]
                    
                    # Calculate center of bounding box
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)
                    
                    # Get depth at center (with small averaging window)
                    depth_window = depth_image[
                        max(0, center_y-5):min(depth_image.shape[0], center_y+5),
                        max(0, center_x-5):min(depth_image.shape[1], center_x+5)
                    ]
                    
                    # Use median depth to be robust to noise
                    valid_depths = depth_window[depth_window > 0]
                    if len(valid_depths) == 0:
                        continue
                    
                    depth_value = np.median(valid_depths)
                    
                    # Deproject to 3D
                    point_3d = self.deproject_pixel_to_point(center_x, center_y, depth_value)
                    if point_3d is None:
                        continue
                    
                    x, y, z = point_3d
                    
                    # Draw on debug image
                    cv2.rectangle(debug_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    label = f'{class_name} {conf:.2f} | {z:.2f}m'
                    cv2.putText(debug_image, label, (int(x1), int(y1)-10),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.circle(debug_image, (center_x, center_y), 5, (0, 0, 255), -1)
                    
                    # Publish pose for target objects
                    if 'charger' in class_name.lower() or 'handle' in class_name.lower():
                        self.publish_detection('charger', x, y, z, rgb_msg.header)
                    elif 'port' in class_name.lower() or 'car' in class_name.lower():
                        self.publish_detection('port', x, y, z, rgb_msg.header)
                    
                    self.get_logger().info(f'Detected {class_name} at ({x:.2f}, {y:.2f}, {z:.2f})')
            
            # Publish debug image
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = rgb_msg.header
            self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in detection: {e}')

    def publish_detection(self, object_type, x, y, z, header):
        """Publish detected object pose."""
        pose_msg = PoseStamped()
        pose_msg.header = header
        pose_msg.header.frame_id = 'camera_color_optical_frame'
        
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.w = 1.0
        
        if object_type == 'charger':
            self.charger_pose_pub.publish(pose_msg)
        elif object_type == 'port':
            self.port_pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = YoloDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
