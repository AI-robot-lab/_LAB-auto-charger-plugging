# Unitree G1 Auto Charger Plugging

Complete software solution for the Unitree G1 EDU humanoid robot to autonomously charge electric vehicles. The robot approaches a charging station, retrieves the charger, navigates to the car, and precisely inserts the plug into the charging port. This project encompasses the entire process: from navigation and computer vision to advanced humanoid arm manipulation.

## Overview

This ROS2-based system enables the Unitree G1 humanoid robot to perform autonomous electric vehicle charging. The robot executes a mission through seven main states:

1. **IDLE** - Wait for a mission start command
2. **NAV_TO_STATION** - Navigate to the charging station
3. **ALIGN_WITH_CHARGER** - Align the base with the charger handle
4. **GRASP_HANDLE** - Detect and grasp the charger handle
5. **NAV_TO_CAR** - Navigate to the vehicle while carrying the charger
6. **ALIGN_WITH_PORT** - Align the arm with the charging port
7. **INSERT_PLUG** - Precisely insert the charger plug into the vehicle's charging port

## System Architecture

The system consists of four main ROS2 packages:

### 1. g1_perception Package
Handles computer vision for detecting the charger handle and car charging port.

**Nodes:**
- `charger_detector` - Detects and localizes the charging handle
- `port_detector` - Detects and localizes the car's charging port

**Topics:**
- Publishes: `/perception/charger_pose`, `/perception/port_pose`
- Subscribes: `/camera/image_raw`

### 2. g1_navigation Package
Manages walking logic and path planning for the humanoid robot.

**Nodes:**
- `walking_controller` - Controls the robot's walking gait and movement
- `path_planner` - Plans collision-free paths

**Topics:**
- Publishes: `/cmd_vel`, `/navigation/planned_path`
- Subscribes: `/odom`, `/navigation/goal`

### 3. g1_manipulation Package
Controls arm inverse kinematics for grasping and insertion operations.

**Nodes:**
- `arm_controller` - Manages arm movements using IK
- `gripper_controller` - Controls gripper open/close operations

**Topics:**
- Publishes: `/right_arm/joint_trajectory`, `/right_gripper/position_command`
- Subscribes: `/joint_states`, `/manipulation/right_arm/target_pose`

### 4. g1_mission_control Package
Coordinates the entire mission using a finite state machine.

**Nodes:**
- `state_machine` - Main coordinator that orchestrates all subsystems

**Topics:**
- Publishes: `/mission_control/state`, `/navigation/goal`, `/manipulation/right_arm/target_pose`
- Subscribes: `/perception/charger_pose`, `/perception/port_pose`, `/navigation/status`

## Installation

### Prerequisites
- ROS2 Humble (or later)
- Ubuntu 22.04 (or compatible)
- Unitree SDK for G1 robot
- Python 3.10+
- OpenCV and cv_bridge

### Building the Workspace

```bash
# Create a workspace
mkdir -p ~/unitree_ws/src
cd ~/unitree_ws/src

# Clone this repository
git clone https://github.com/AI-robot-lab/unitree-g1-auto-charger-plugging.git

# Install dependencies
cd ~/unitree_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install

# Source the workspace
source ~/unitree_ws/install/setup.bash
```

## Usage

### Launching the System

Launch all nodes with default configuration:
```bash
ros2 launch launch/bringup.launch.py
```

Launch with auto-start enabled:
```bash
ros2 launch launch/bringup.launch.py auto_start:=true
```

Launch with custom configuration:
```bash
ros2 launch launch/bringup.launch.py config_file:=/path/to/custom_config.yaml
```

### Starting the Mission

If not using auto-start, trigger the mission manually:
```bash
ros2 topic pub /mission_control/start std_msgs/Bool "data: true" --once
```

### Monitoring the System

Monitor the current mission state:
```bash
ros2 topic echo /mission_control/state
```

View perception results:
```bash
ros2 topic echo /perception/charger_pose
ros2 topic echo /perception/port_pose
```

Check gripper status:
```bash
ros2 topic echo /manipulation/right_gripper/grasp_status
```

## Configuration

All parameters are configured in `config/g1_charging_params.yaml`. Key parameters include:

- **Perception**: Camera topics, detection thresholds, model paths
- **Navigation**: Velocity limits, position tolerances, planning algorithms
- **Manipulation**: Arm control rates, force limits, gripper positions
- **Mission Control**: Station/car positions, state timeouts

## Unitree SDK Integration

This system integrates with the Unitree SDK to control the G1 robot's low-level functions:

### SDK Setup

1. Install the Unitree SDK:
```bash
# Follow Unitree's official SDK installation guide
# Typically involves:
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2
mkdir build && cd build
cmake ..
make
sudo make install
```

2. Configure network settings in `config/g1_charging_params.yaml`:
```yaml
    unitree_sdk:
      ros__parameters:
        robot_ip: "192.168.123.10"     # Robot's IP address
        local_ip: "192.168.123.100"    # Your computer's IP
        sdk_port: 8080
```

### SDK Integration Points

The system interfaces with the Unitree SDK at several points:

- **Walking Controller**: Uses SDK's locomotion interface for gait control
- **Arm Controller**: Interfaces with SDK's arm control for joint commands
- **Gripper Controller**: Uses SDK's gripper interface for grasp control
- **State Machine**: Monitors robot state via SDK status messages

### Safety Features

The SDK integration includes safety mechanisms:
- Emergency stop functionality
- Joint limit enforcement
- Collision detection
- Balance monitoring
- Battery level checks

## Development

### Package Structure

```
unitree-g1-auto-charger-plugging/
├── g1_perception/
│   ├── g1_perception/
│   │   ├── __init__.py
│   │   ├── charger_detector.py
│   │   └── port_detector.py
│   ├── package.xml
│   └── setup.py
├── g1_navigation/
│   ├── g1_navigation/
│   │   ├── __init__.py
│   │   ├── walking_controller.py
│   │   └── path_planner.py
│   ├── package.xml
│   └── setup.py
├── g1_manipulation/
│   ├── g1_manipulation/
│   │   ├── __init__.py
│   │   ├── arm_controller.py
│   │   └── gripper_controller.py
│   ├── package.xml
│   └── setup.py
├── g1_mission_control/
│   ├── g1_mission_control/
│   │   ├── __init__.py
│   │   └── state_machine.py
│   ├── package.xml
│   └── setup.py
├── launch/
│   └── bringup.launch.py
├── config/
│   ├── g1_charging_params.yaml
│   └── g1_params.yaml
└── README.md
```

### Adding Custom Behaviors

To extend the system with custom behaviors:

1. **New Perception Algorithms**: Add to g1_perception package
2. **Alternative Path Planning**: Modify g1_navigation package
3. **Advanced Manipulation**: Extend g1_manipulation package
4. **Additional Mission States**: Update state machine in g1_mission_control

## Troubleshooting

### Common Issues

1. **Robot not responding**
   - Check network connectivity to robot
   - Verify SDK IP configuration
   - Ensure SDK is properly installed

2. **Perception not detecting objects**
   - Check camera topic names
   - Verify detection thresholds
   - Ensure adequate lighting

3. **Navigation issues**
   - Check odometry data
   - Verify position tolerances
   - Review path planning parameters

4. **Manipulation problems**
   - Verify IK solver configuration
   - Check joint limits
   - Ensure proper gripper calibration

## Contributing

Contributions are welcome! Please follow these guidelines:
1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- Unitree Robotics for the G1 robot platform and SDK
- ROS2 community for the excellent robotics middleware
- Contributors and maintainers of this project

## Contact

For questions, issues, or suggestions, please open an issue on GitHub or contact the maintainers.

## References

- [Unitree G1 Robot](https://www.unitree.com/)
- [Unitree SDK Documentation](https://github.com/unitreerobotics/unitree_sdk2)
- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [OpenCV Documentation](https://docs.opencv.org/)
