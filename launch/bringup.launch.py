#!/usr/bin/env python3
"""
Bringup Launch File
Launches all nodes for the Unitree G1 charger plugging system.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for the charger plugging system."""
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='config/g1_charging_params.yaml',
        description='Path to the configuration file'
    )
    
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='Automatically start the mission on launch'
    )
    
    # Get launch configurations
    config_file = LaunchConfiguration('config_file')
    auto_start = LaunchConfiguration('auto_start')
    
    # Perception nodes
    charger_detector_node = Node(
        package='g1_perception',
        executable='charger_detector',
        name='charger_detector',
        output='screen',
        parameters=[config_file]
    )
    
    port_detector_node = Node(
        package='g1_perception',
        executable='port_detector',
        name='port_detector',
        output='screen',
        parameters=[config_file]
    )
    
    # Navigation nodes
    walking_controller_node = Node(
        package='g1_navigation',
        executable='walking_controller',
        name='walking_controller',
        output='screen',
        parameters=[config_file]
    )
    
    path_planner_node = Node(
        package='g1_navigation',
        executable='path_planner',
        name='path_planner',
        output='screen',
        parameters=[config_file]
    )
    
    # Manipulation nodes
    arm_controller_node = Node(
        package='g1_manipulation',
        executable='arm_controller',
        name='arm_controller',
        output='screen',
        parameters=[config_file]
    )
    
    gripper_controller_node = Node(
        package='g1_manipulation',
        executable='gripper_controller',
        name='gripper_controller',
        output='screen',
        parameters=[config_file]
    )
    
    # Mission control node
    state_machine_node = Node(
        package='g1_mission_control',
        executable='state_machine',
        name='state_machine',
        output='screen',
        parameters=[
            config_file,
            {'auto_start': auto_start}
        ]
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(config_file_arg)
    ld.add_action(auto_start_arg)
    
    # Add nodes
    ld.add_action(charger_detector_node)
    ld.add_action(port_detector_node)
    ld.add_action(walking_controller_node)
    ld.add_action(path_planner_node)
    ld.add_action(arm_controller_node)
    ld.add_action(gripper_controller_node)
    ld.add_action(state_machine_node)
    
    return ld
