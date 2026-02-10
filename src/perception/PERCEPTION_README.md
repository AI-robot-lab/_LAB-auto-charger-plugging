# Perception Package

Expanded perception capabilities for the Unitree G1 robot with LIDAR safety monitoring and dual vision strategies.

## Features

### 1. LIDAR Safety Node
Monitors LIDAR data for obstacles and triggers emergency stop when needed.

**Functionality:**
- Continuously monitors `/scan` topic
- Checks for obstacles within configurable safety distance
- Monitors a front arc (default 60°)
- Publishes to `/safety/emergency_stop` when obstacles detected

**Configuration:**
```yaml
lidar_safety_node:
  ros__parameters:
    scan_topic: "/scan"
    safety_distance: 0.5  # meters
    front_arc_degrees: 60.0  # degrees
    check_rate: 10.0  # Hz
```

### 2. Vision Strategy A: YOLOv8 + Depth (GPU Heavy)

High-accuracy object detection using YOLOv8 with RGB-Depth fusion.

**Features:**
- YOLOv8 object detection (nano to extra-large models)
- Synchronized RGB + Depth processing
- 3D position estimation using pinhole camera model
- GPU-accelerated inference
- Debug visualization

**Models Available:**
- `yolov8n.pt` - Nano (fastest, lowest accuracy)
- `yolov8s.pt` - Small
- `yolov8m.pt` - Medium
- `yolov8l.pt` - Large
- `yolov8x.pt` - Extra Large (slowest, highest accuracy)

**Installation:**
```bash
pip install ultralytics
```

**Usage:**
```bash
ros2 launch launch/perception.launch.py detection_mode:=yolo
```

### 3. Vision Strategy B: MediaPipe (CPU Friendly)

Efficient object detection using Google MediaPipe for edge devices.

**Features:**
- CPU-optimized inference
- Lower power consumption
- Suitable for embedded systems
- Fallback color-based detection if model unavailable
- Same 3D mapping as YOLO strategy

**Installation:**
```bash
pip install mediapipe
```

**Usage:**
```bash
ros2 launch launch/perception.launch.py detection_mode:=mediapipe
```

## Launch System

The perception launch file (`launch/perception.launch.py`) conditionally starts detection nodes based on configuration:

```bash
# Use MediaPipe (default)
ros2 launch launch/perception.launch.py

# Use YOLOv8
ros2 launch launch/perception.launch.py detection_mode:=yolo

# Use specific config file
ros2 launch launch/perception.launch.py config_file:=/path/to/config.yaml

# Auto-detect from config
ros2 launch launch/perception.launch.py detection_mode:=auto
```

**Note:** LIDAR safety node is ALWAYS launched regardless of detection mode.

## Topics

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `/safety/emergency_stop` | std_msgs/Bool | Emergency stop signal from LIDAR |
| `/perception/charger_pose` | geometry_msgs/PoseStamped | Detected charger handle 3D position |
| `/perception/port_pose` | geometry_msgs/PoseStamped | Detected charging port 3D position |
| `/perception/debug_image` | sensor_msgs/Image | Visualization with bounding boxes |

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | sensor_msgs/LaserScan | LIDAR data |
| `/camera/color/image_raw` | sensor_msgs/Image | RGB camera feed |
| `/camera/aligned_depth_to_color/image_raw` | sensor_msgs/Image | Aligned depth map |
| `/camera/color/camera_info` | sensor_msgs/CameraInfo | Camera intrinsics |

## 3D Mapping

Both vision strategies use the **Pinhole Camera Model** to deproject 2D detections to 3D:

```
X = (pixel_x - cx) * depth / fx
Y = (pixel_y - cy) * depth / fy
Z = depth
```

Where:
- `(cx, cy)` - Principal point from camera_info
- `(fx, fy)` - Focal lengths from camera_info
- `depth` - Depth value at pixel location (meters)

## Camera Setup

**Supported Cameras:**
- Intel RealSense D435i/D455
- Unitree Integrated Depth Camera
- Any RGB-D camera with aligned depth output

**Requirements:**
- RGB and Depth topics must be synchronized
- Camera must publish `camera_info` with intrinsics
- Depth should be aligned to RGB frame

**RealSense Setup:**
```bash
# Install RealSense ROS wrapper
sudo apt install ros-${ROS_DISTRO}-realsense2-camera

# Launch camera
ros2 launch realsense2_camera rs_launch.py \
    align_depth.enable:=true \
    enable_color:=true \
    enable_depth:=true
```

## Configuration

Main config file: `config/perception_params.yaml`

### Key Parameters

```yaml
# Select detection mode
detection_mode: 'mediapipe'  # or 'yolo'

# LIDAR Safety
lidar_safety_node:
  ros__parameters:
    safety_distance: 0.5  # meters
    front_arc_degrees: 60.0

# YOLO Detector
yolo_detector_node:
  ros__parameters:
    model_path: "yolov8n.pt"
    confidence_threshold: 0.5
    use_gpu: true

# MediaPipe Detector  
mediapipe_detector_node:
  ros__parameters:
    model_path: "efficientdet_lite0.tflite"
    confidence_threshold: 0.5
    max_results: 5
```

## Performance Comparison

| Strategy | Platform | FPS | Power | Accuracy |
|----------|----------|-----|-------|----------|
| YOLOv8n (GPU) | RTX 3060 | 60+ | High | Good |
| YOLOv8m (GPU) | RTX 3060 | 30-40 | High | Excellent |
| MediaPipe (CPU) | i5-8250U | 15-20 | Low | Good |
| MediaPipe (CPU) | Jetson Nano | 10-15 | Very Low | Good |

## Troubleshooting

### LIDAR Safety

**Problem:** No emergency stop signals  
**Solution:**  
- Check LIDAR is publishing to `/scan`
- Verify safety_distance and front_arc settings
- Check LIDAR is calibrated correctly

### YOLO Detection

**Problem:** CUDA out of memory  
**Solution:**
- Use smaller model (yolov8n.pt instead of yolov8x.pt)
- Reduce image resolution
- Switch to MediaPipe

**Problem:** ultralytics not found  
**Solution:**
```bash
pip install ultralytics
```

### MediaPipe Detection

**Problem:** mediapipe not found  
**Solution:**
```bash
pip install mediapipe
```

**Problem:** Model file not found  
**Solution:**
- MediaPipe will fallback to color-based detection
- Download model from MediaPipe website
- Update `model_path` in config

### Depth Integration

**Problem:** Objects detected but no 3D pose published  
**Solution:**
- Check depth image is aligned to RGB
- Verify camera_info is being published
- Check for invalid depth values (inf, nan, 0)

## Development

### Adding Custom Detectors

1. Create new node in `perception/perception/`
2. Subscribe to RGB + Depth with message_filters
3. Implement detection logic
4. Use `deproject_pixel_to_point()` for 3D mapping
5. Publish to `/perception/charger_pose` or `/perception/port_pose`
6. Add entry point to `setup.py`
7. Update launch file to include new mode

### Testing Individual Nodes

```bash
# Test LIDAR safety
ros2 run perception lidar_safety_node

# Test YOLO detector
ros2 run perception yolo_detector_node

# Test MediaPipe detector
ros2 run perception mediapipe_detector_node
```

### Viewing Debug Images

```bash
# Use rqt_image_view
ros2 run rqt_image_view rqt_image_view /perception/debug_image

# Or use image_view
ros2 run image_view image_view --ros-args -r image:=/perception/debug_image
```

## Safety Considerations

- **LIDAR safety node must be running** during robot operation
- Emergency stop should be wired to robot's hardware E-stop
- Test thoroughly in simulation before hardware deployment
- Maintain clear LIDAR field of view
- Calibrate safety distances for your specific environment

## Future Enhancements

- [ ] Multi-camera fusion
- [ ] IMU integration for motion compensation
- [ ] Temporal filtering for stable detections
- [ ] Custom trained models for charger/port
- [ ] ArUco marker detection for precise alignment
- [ ] 6D pose estimation (not just 3D position)
- [ ] Real-time SLAM integration

## References

- [YOLOv8 Documentation](https://docs.ultralytics.com/)
- [MediaPipe Documentation](https://developers.google.com/mediapipe)
- [RealSense ROS](https://github.com/IntelRealSense/realsense-ros)
- [ROS 2 Humble](https://docs.ros.org/en/humble/)
