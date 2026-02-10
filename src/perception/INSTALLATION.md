# Perception Enhancement - Installation Guide

## Quick Start

### 1. Prerequisites

```bash
# Install ROS 2 dependencies
sudo apt install ros-humble-cv-bridge \
                 ros-humble-image-transport \
                 ros-humble-message-filters \
                 python3-opencv \
                 python3-numpy

# For YOLO support (GPU required)
pip install ultralytics --break-system-packages

# For MediaPipe support (CPU-friendly)
pip install mediapipe --break-system-packages
```

### 2. Integration into Existing Project

Copy the files to your ROS2 workspace:

```bash
# Assuming your workspace is ~/unitree_ws
cd ~/unitree_ws

# Copy perception package (will overwrite existing)
cp -r perception_enhancement/src/perception src/

# Copy config (merge with existing config/g1_params.yaml)
cp perception_enhancement/config/perception_params.yaml config/

# Copy launch file
cp perception_enhancement/launch/perception.launch.py launch/
```

### 3. Build

```bash
cd ~/unitree_ws
colcon build --packages-select perception
source install/setup.bash
```

### 4. Test Individual Nodes

```bash
# Test LIDAR safety node
ros2 run perception lidar_safety_node

# Test YOLO detector (requires GPU and ultralytics)
ros2 run perception yolo_detector_node

# Test MediaPipe detector (CPU-friendly)
ros2 run perception mediapipe_detector_node
```

### 5. Launch Full Perception System

```bash
# Launch with default settings (MediaPipe)
ros2 launch launch/perception.launch.py

# Launch with YOLO (GPU required)
ros2 launch launch/perception.launch.py detection_mode:=yolo

# Launch with custom config
ros2 launch launch/perception.launch.py config_file:=/path/to/config.yaml
```

## Configuration

Edit `config/perception_params.yaml` to customize:

```yaml
# Choose detection strategy
detection_mode: 'mediapipe'  # or 'yolo'

# LIDAR safety parameters
lidar_safety_node:
  ros__parameters:
    safety_distance: 0.5  # meters
    front_arc_degrees: 60.0

# YOLO parameters (if using GPU strategy)
yolo_detector_node:
  ros__parameters:
    model_path: "yolov8n.pt"  # Options: n, s, m, l, x
    confidence_threshold: 0.5
    use_gpu: true

# MediaPipe parameters (if using CPU strategy)
mediapipe_detector_node:
  ros__parameters:
    confidence_threshold: 0.5
    max_results: 5
```

## Camera Setup

### Intel RealSense D435i/D455

```bash
# Install RealSense ROS2 wrapper
sudo apt install ros-humble-realsense2-camera

# Launch camera with depth alignment
ros2 launch realsense2_camera rs_launch.py \
    align_depth.enable:=true \
    enable_color:=true \
    enable_depth:=true \
    depth_module.profile:=640x480x30 \
    rgb_camera.profile:=640x480x30
```

### Unitree Integrated Camera

Configure the camera topics in `perception_params.yaml`:

```yaml
yolo_detector_node:
  ros__parameters:
    rgb_topic: "/unitree_camera/color/image_raw"
    depth_topic: "/unitree_camera/aligned_depth/image_raw"
    camera_info_topic: "/unitree_camera/color/camera_info"
```

## Verification

### Check Nodes

```bash
# List running nodes
ros2 node list

# Should see:
# /lidar_safety_node
# /yolo_detector_node OR /mediapipe_detector_node
```

### Monitor Topics

```bash
# Check LIDAR safety
ros2 topic echo /safety/emergency_stop

# Check detections
ros2 topic echo /perception/charger_pose
ros2 topic echo /perception/port_pose

# Monitor detection rate
ros2 topic hz /perception/charger_pose
```

### View Debug Images

```bash
# Using image_view
ros2 run image_view image_view --ros-args -r image:=/perception/debug_image

# Or using rqt
rqt_image_view /perception/debug_image
```

## Troubleshooting

### "ultralytics not found"

```bash
pip install ultralytics --break-system-packages
```

### "mediapipe not found"

```bash
pip install mediapipe --break-system-packages
```

### "CUDA out of memory" (YOLO)

Solutions:
1. Use smaller model: `yolov8n.pt` instead of `yolov8x.pt`
2. Switch to MediaPipe: `detection_mode:=mediapipe`
3. Reduce image resolution in camera driver

### "No depth data"

Checklist:
- Camera publishing to `/camera/aligned_depth_to_color/image_raw`
- Depth alignment enabled in camera launch
- camera_info topic publishing intrinsics
- Depth values in valid range (0.1m - 10m)

### "No detections"

Checklist:
- Camera topics configured correctly
- Objects in camera field of view
- Sufficient lighting
- Confidence threshold not too high
- Check debug image for bounding boxes

## Integration with Mission Control

The perception system integrates with the existing mission_control action client:

```python
# mission_control already subscribes to:
# - /perception/charger_pose
# - /perception/port_pose
# - /safety/emergency_stop

# These topics are now published by:
# - yolo_detector_node OR mediapipe_detector_node
# - lidar_safety_node
```

No changes needed to mission_control!

## Performance Tips

### For GPU Systems (NVIDIA)

Use YOLO for best accuracy:
```yaml
detection_mode: 'yolo'
yolo_detector_node:
  ros__parameters:
    model_path: "yolov8m.pt"  # Medium model
    use_gpu: true
```

### For CPU/Embedded Systems

Use MediaPipe for efficiency:
```yaml
detection_mode: 'mediapipe'
mediapipe_detector_node:
  ros__parameters:
    max_results: 3  # Reduce processing
```

### For Jetson Nano/Xavier

Either strategy works:
- YOLO: Use yolov8n.pt (nano model)
- MediaPipe: Works well out of the box

## Next Steps

1. Install and test perception system
2. Configure camera parameters
3. Tune detection thresholds
4. Test LIDAR safety distances
5. Integrate with full mission system
6. Deploy to Unitree G1 robot
7. Test in target environment

## Documentation

- **PERCEPTION_README.md** - Detailed package documentation
- **PERCEPTION_ENHANCEMENT_SUMMARY.md** - Implementation details
- **STRUCTURE.txt** - File organization

## Support

For issues or questions:
1. Check docs/PERCEPTION_README.md troubleshooting section
2. Verify all dependencies installed
3. Check ROS2 logs: `ros2 topic echo /rosout`
4. Test nodes individually before full system
