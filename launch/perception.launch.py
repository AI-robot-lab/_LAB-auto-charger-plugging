#!/usr/bin/env python3
"""
Perception Launch File
Conditionally launches detection nodes based on configuration.
Always launches LIDAR safety node.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def load_yaml(context, config_file_path):
    """Load YAML configuration file."""
    try:
        with open(config_file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        print(f"Error loading config file: {e}")
        return {}


def launch_setup(context, *args, **kwargs):
    """Setup function to conditionally launch nodes based on config."""
    
    # Get launch configurations
    config_file = LaunchConfiguration('config_file').perform(context)
    detection_mode = LaunchConfiguration('detection_mode').perform(context)
    
    # Load config file to get default detection_mode if not specified
    config = load_yaml(context, config_file)
    if detection_mode == 'auto':
        detection_mode = config.get('detection_mode', 'mediapipe')
    
    print(f"=" * 70)
    print(f"Perception Launch Configuration")
    print(f"=" * 70)
    print(f"Config file: {config_file}")
    print(f"Detection mode: {detection_mode}")
    print(f"=" * 70)
    
    nodes_to_launch = []
    
    # 1. Always launch LIDAR Safety Node
    lidar_safety_node = Node(
        package='perception',
        executable='lidar_safety_node',
        name='lidar_safety_node',
        output='screen',
        parameters=[config_file]
    )
    nodes_to_launch.append(lidar_safety_node)
    print("✓ LIDAR Safety Node: ENABLED")
    
    # 2. Conditionally launch detection node
    if detection_mode == 'yolo':
        yolo_node = Node(
            package='perception',
            executable='yolo_detector_node',
            name='yolo_detector_node',
            output='screen',
            parameters=[config_file]
        )
        nodes_to_launch.append(yolo_node)
        print("✓ YOLOv8 Detector: ENABLED (GPU-heavy)")
        print("✗ MediaPipe Detector: DISABLED")
        
    elif detection_mode == 'mediapipe':
        mediapipe_node = Node(
            package='perception',
            executable='mediapipe_detector_node',
            name='mediapipe_detector_node',
            output='screen',
            parameters=[config_file]
        )
        nodes_to_launch.append(mediapipe_node)
        print("✗ YOLOv8 Detector: DISABLED")
        print("✓ MediaPipe Detector: ENABLED (CPU-friendly)")
        
    else:
        print(f"WARNING: Unknown detection mode '{detection_mode}'")
        print("Valid options: 'yolo' or 'mediapipe'")
        print("Defaulting to MediaPipe...")
        mediapipe_node = Node(
            package='perception',
            executable='mediapipe_detector_node',
            name='mediapipe_detector_node',
            output='screen',
            parameters=[config_file]
        )
        nodes_to_launch.append(mediapipe_node)
    
    print(f"=" * 70)
    
    return nodes_to_launch


def generate_launch_description():
    """Generate launch description for perception system."""
    
    # Find the perception package
    perception_share = FindPackageShare('perception')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('perception'),
            '..',
            '..',
            '..',
            'config',
            'perception_params.yaml'
        ]),
        description='Path to the perception configuration file'
    )
    
    detection_mode_arg = DeclareLaunchArgument(
        'detection_mode',
        default_value='auto',
        description="Detection mode: 'yolo', 'mediapipe', or 'auto' (reads from config)"
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(config_file_arg)
    ld.add_action(detection_mode_arg)
    
    # Add opaque function to conditionally launch nodes
    ld.add_action(OpaqueFunction(function=launch_setup))
    
    return ld
