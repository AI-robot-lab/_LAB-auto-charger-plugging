# Unitree G1 Auto Charger Plugging

Complete software solution for the Unitree G1 EDU humanoid robot to autonomously charge electric vehicles. The robot approaches a charging station, retrieves the charger, navigates to the car, and precisely inserts the plug into the charging port. This project encompasses the entire process: from navigation and computer vision to advanced humanoid arm manipulation.

## Overview

This ROS2-based system enables the Unitree G1 humanoid robot to perform autonomous electric vehicle charging. The robot executes a mission through four main states:

1. **APPROACH_STATION** - Navigate to the charging station
2. **GRASP_CHARGER** - Detect and grasp the charger handle
3. **APPROACH_CAR** - Navigate to the vehicle while carrying the charger
4. **INSERT_PLUG** - Precisely insert the charger plug into the vehicle's charging port

## System Architecture

The system uses **ROS2 Actions** for robust, asynchronous communication between subsystems. It consists of five main ROS2 packages:

### 0. Charging Interfaces Package
Defines custom action interfaces for the system.

**Actions:**
- `Navigate.action` - Navigation goals with pose targets
- `Manipulate.action` - Manipulation tasks (grasp_handle, insert_plug)
- `Detect.action` - Object detection requests

### 1. Perception Package
Handles computer vision for detecting the charger handle and car charging port.

**Nodes:**
- `charger_detector` - Detects and localizes the charging handle
- `port_detector` - Detects and localizes the car's charging port
- `perception_action_server` - **Action Server** for object detection

**Actions:**
- Serves: `/detect` action (Detect.action)

**Topics:**
- Publishes: `/perception/charger_pose`, `/perception/port_pose`
- Subscribes: `/camera/image_raw`

### 2. Navigation Package
Manages walking logic and path planning for the humanoid robot.

**Nodes:**
- `walking_controller` - Controls the robot's walking gait and movement
- `path_planner` - Plans collision-free paths
- `navigation_action_server` - **Action Server** for navigation tasks

**Actions:**
- Serves: `/navigate` action (Navigate.action)

**Topics:**
- Publishes: `/cmd_vel`, `/navigation/planned_path`
- Subscribes: `/odom`, `/navigation/goal`

### 3. Manipulation Package
Controls arm inverse kinematics for grasping and insertion operations.

**Nodes:**
- `arm_controller` - Manages arm movements using IK
- `gripper_controller` - Controls gripper open/close operations
- `manipulation_action_server` - **Action Server** for manipulation tasks

**Actions:**
- Serves: `/manipulate` action (Manipulate.action)

**Topics:**
- Publishes: `/right_arm/joint_trajectory`, `/right_gripper/position_command`
- Subscribes: `/joint_states`, `/manipulation/right_arm/target_pose`

### 4. Mission Control Package
Coordinates the entire mission using a finite state machine acting as an **Action Client**.

**Nodes:**
- `state_machine` - Original topic-based coordinator (deprecated)
- `state_machine_action_client` - **Main coordinator using ROS2 Actions**

**Mission Sequence:**
1. `NAV_TO_STATION` - Navigate to charging station
2. `DETECT_HANDLE` - Detect charger handle
3. `MANIPULATE_GRASP` - Grasp the handle
4. `NAV_TO_CAR` - Navigate to car
5. `DETECT_PORT` - Detect charging port
6. `MANIPULATE_INSERT` - Insert plug into port

**Actions (as Client):**
- Calls: `/navigate`, `/manipulate`, `/detect` actions

**Topics:**
- Publishes: `/mission_control/state`

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

#### Using Action-Based Architecture (Recommended)

To test the action-based system with mock servers:

```bash
# Terminal 1 - Start Navigation Action Server
ros2 run navigation navigation_action_server

# Terminal 2 - Start Manipulation Action Server
ros2 run manipulation manipulation_action_server

# Terminal 3 - Start Perception Action Server
ros2 run perception perception_action_server

# Terminal 4 - Start Mission Control (Action Client)
ros2 run mission_control state_machine_action_client
```

The mission will automatically start and execute through all states.

#### Legacy Launch (Topic-Based)

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

### Testing the Action-Based System

Validate the architecture:
```bash
python3 validate_architecture.py
```

Monitor action servers:
```bash
# List available actions
ros2 action list

# Get info about a specific action
ros2 action info /navigate
ros2 action info /manipulate
ros2 action info /detect
```

Send a test goal to an action server:
```bash
# Test navigation
ros2 action send_goal /navigate charging_interfaces/action/Navigate "{target_pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"

# Test manipulation
ros2 action send_goal /manipulate charging_interfaces/action/Manipulate "{task_type: 'grasp_handle'}"

# Test detection
ros2 action send_goal /detect charging_interfaces/action/Detect "{object_name: 'charger_handle'}"
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

All parameters are configured in `config/g1_params.yaml`. Key parameters include:

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

2. Configure network settings in `config/g1_params.yaml`:
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
├── src/
│   ├── charging_interfaces/         # NEW: Action interface definitions
│   │   ├── action/
│   │   │   ├── Navigate.action
│   │   │   ├── Manipulate.action
│   │   │   └── Detect.action
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── perception/
│   │   ├── perception/
│   │   │   ├── __init__.py
│   │   │   ├── charger_detector.py
│   │   │   ├── port_detector.py
│   │   │   └── perception_action_server.py  # NEW: Action server
│   │   ├── package.xml
│   │   └── setup.py
│   ├── navigation/
│   │   ├── navigation/
│   │   │   ├── __init__.py
│   │   │   ├── walking_controller.py
│   │   │   ├── path_planner.py
│   │   │   └── navigation_action_server.py  # NEW: Action server
│   │   ├── package.xml
│   │   └── setup.py
│   ├── manipulation/
│   │   ├── manipulation/
│   │   │   ├── __init__.py
│   │   │   ├── arm_controller.py
│   │   │   ├── gripper_controller.py
│   │   │   └── manipulation_action_server.py  # NEW: Action server
│   │   ├── package.xml
│   │   └── setup.py
│   └── mission_control/
│       ├── mission_control/
│       │   ├── __init__.py
│       │   ├── state_machine.py
│       │   └── state_machine_action_client.py  # NEW: Action client
│       ├── package.xml
│       └── setup.py
├── launch/
│   └── bringup.launch.py
├── config/
│   └── g1_params.yaml
├── validate_architecture.py     # NEW: Validation script
└── README.md
```

### Adding Custom Behaviors

To extend the system with custom behaviors:

1. **New Perception Algorithms**: Add to perception package
2. **Alternative Path Planning**: Modify navigation package
3. **Advanced Manipulation**: Extend manipulation package
4. **Additional Mission States**: Update state machine action client in mission_control
5. **New Action Types**: Add action definitions to charging_interfaces package

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
