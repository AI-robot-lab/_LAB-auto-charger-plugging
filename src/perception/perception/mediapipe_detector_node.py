#!/usr/bin/env python3
"""
MediaPipe Detector Node
Uses MediaPipe Object Detection for CPU-friendly detection with depth integration.
Optimized for edge devices and lower-power systems.
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
    import mediapipe as mp
    from mediapipe.tasks import python
    from mediapipe.tasks.python import vision
    MEDIAPIPE_AVAILABLE = True
except ImportError:
    MEDIAPIPE_AVAILABLE = False


class MediaPipeDetectorNode(Node):
    """Node for MediaPipe-based object detection with depth integration."""

    def __init__(self):
        super().__init__('mediapipe_detector_node')
        
        if not MEDIAPIPE_AVAILABLE:
            self.get_logger().error('mediapipe not installed! Run: pip install mediapipe')
            raise ImportError('mediapipe package required')
        
        # Declare parameters
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('model_path', 'efficientdet_lite0.tflite')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('max_results', 5)
        
        # Get parameters
        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        max_results = self.get_parameter('max_results').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Camera intrinsics
        self.camera_info = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        
        # Initialize MediaPipe Object Detector
        self.get_logger().info(f'Loading MediaPipe model: {model_path}')
        try:
            base_options = python.BaseOptions(model_asset_path=model_path)
            options = vision.ObjectDetectorOptions(
                base_options=base_options,
                score_threshold=self.confidence_threshold,
                max_results=max_results,
                running_mode=vision.RunningMode.IMAGE
            )
            self.detector = vision.ObjectDetector.create_from_options(options)
            self.get_logger().info('MediaPipe model loaded successfully (CPU)')
        except Exception as e:
            self.get_logger().warn(f'Could not load model from {model_path}: {e}')
            self.get_logger().warn('Will use fallback detection mode')
            self.detector = None
        
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
        
        # Debug image publisher
        self.debug_image_pub = self.create_publisher(
            Image,
            '/perception/debug_image',
            10
        )
        
        # Fallback: simple color-based detection if model not available
        self.use_fallback = (self.detector is None)
        
        self.get_logger().info('MediaPipe Detector Node initialized')

    def camera_info_callback(self, msg):
        """Store camera intrinsics from camera_info."""
        if self.camera_info is None:
            self.camera_info = msg
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

    def fallback_color_detection(self, rgb_image, depth_image):
        """
        Simple fallback detection using color segmentation.
        Looks for distinctive colors that might indicate charger or port.
        """
        detections = []
        
        # Convert to HSV for better color segmentation
        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
        
        # Define color ranges (example: looking for red/orange charger handle)
        # Lower red (0-10)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        # Upper red (170-180)
        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        
        # Create masks
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 | mask2
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Minimum area threshold
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w // 2
                center_y = y + h // 2
                
                detections.append({
                    'bbox': (x, y, x+w, y+h),
                    'center': (center_x, center_y),
                    'category': 'charger_handle',
                    'score': 0.6  # Fixed confidence for color detection
                })
        
        return detections

    def synchronized_callback(self, rgb_msg, depth_msg):
        """Process synchronized RGB and Depth images."""
        try:
            # Convert images
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            
            # Convert depth to meters if needed
            if depth_image.dtype == np.uint16:
                depth_image = depth_image.astype(np.float32) / 1000.0
            
            debug_image = rgb_image.copy()
            
            if self.use_fallback:
                # Use fallback color-based detection
                detections = self.fallback_color_detection(rgb_image, depth_image)
            else:
                # Use MediaPipe detection
                # Convert BGR to RGB for MediaPipe
                rgb_for_mp = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_for_mp)
                
                # Run detection
                detection_result = self.detector.detect(mp_image)
                
                # Convert MediaPipe detections to common format
                detections = []
                for detection in detection_result.detections:
                    bbox = detection.bounding_box
                    x1, y1 = bbox.origin_x, bbox.origin_y
                    x2, y2 = x1 + bbox.width, y1 + bbox.height
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)
                    
                    category = detection.categories[0]
                    
                    detections.append({
                        'bbox': (x1, y1, x2, y2),
                        'center': (center_x, center_y),
                        'category': category.category_name,
                        'score': category.score
                    })
            
            # Process all detections
            for det in detections:
                x1, y1, x2, y2 = det['bbox']
                center_x, center_y = det['center']
                category = det['category']
                score = det['score']
                
                # Get depth at center
                depth_window = depth_image[
                    max(0, center_y-5):min(depth_image.shape[0], center_y+5),
                    max(0, center_x-5):min(depth_image.shape[1], center_x+5)
                ]
                
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
                label = f'{category} {score:.2f} | {z:.2f}m'
                cv2.putText(debug_image, label, (int(x1), int(y1)-10),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.circle(debug_image, (center_x, center_y), 5, (0, 0, 255), -1)
                
                # Publish pose for target objects
                if 'charger' in category.lower() or 'handle' in category.lower():
                    self.publish_detection('charger', x, y, z, rgb_msg.header)
                    self.get_logger().info(f'Charger detected at ({x:.2f}, {y:.2f}, {z:.2f})')
                elif 'port' in category.lower() or 'car' in category.lower():
                    self.publish_detection('port', x, y, z, rgb_msg.header)
                    self.get_logger().info(f'Port detected at ({x:.2f}, {y:.2f}, {z:.2f})')
            
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
        node = MediaPipeDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
