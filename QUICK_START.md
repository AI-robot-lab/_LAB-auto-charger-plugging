# Quick Start Guide - ROS2 Action-Based Architecture

## Overview
This guide provides quick commands to get started with the action-based charging system.

## Prerequisites
- ROS2 Humble or later installed
- Workspace built with `colcon build`
- Workspace sourced: `source install/setup.bash`

## Launch Action Servers (Mock Implementation)

Open 4 separate terminals and run:

### Terminal 1 - Navigation Server
```bash
ros2 run navigation navigation_action_server
```

### Terminal 2 - Manipulation Server
```bash
ros2 run manipulation manipulation_action_server
```

### Terminal 3 - Perception Server
```bash
ros2 run perception perception_action_server
```

### Terminal 4 - Mission Control (Action Client)
```bash
ros2 run mission_control state_machine_action_client
```

The mission will start automatically and execute through all states.

## Expected Output

You should see the following sequence in the mission control terminal:

1. `State: NAV_TO_STATION` - Navigating to charging station
2. `State: DETECT_HANDLE` - Detecting charger handle
3. `State: MANIPULATE_GRASP` - Grasping the handle
4. `State: NAV_TO_CAR` - Navigating to car
5. `State: DETECT_PORT` - Detecting charging port
6. `State: MANIPULATE_INSERT` - Inserting plug
7. `MISSION COMPLETED SUCCESSFULLY!`

## Testing Individual Actions

### Test Navigation
```bash
ros2 action send_goal /navigate charging_interfaces/action/Navigate \
  "{target_pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

### Test Manipulation
```bash
ros2 action send_goal /manipulate charging_interfaces/action/Manipulate \
  "{task_type: 'grasp_handle'}"
```

### Test Detection
```bash
ros2 action send_goal /detect charging_interfaces/action/Detect \
  "{object_name: 'charger_handle'}"
```

## Monitoring Tools

### List Available Actions
```bash
ros2 action list
```
Expected output:
- `/detect`
- `/manipulate`
- `/navigate`

### Get Action Info
```bash
ros2 action info /navigate
ros2 action info /manipulate
ros2 action info /detect
```

### Monitor Mission State
```bash
ros2 topic echo /mission_control/state
```

### View Action Feedback (while action is running)
```bash
ros2 topic echo /navigate/_action/feedback
ros2 topic echo /manipulate/_action/feedback
ros2 topic echo /detect/_action/feedback
```

## Validation

Run the validation script to verify the architecture:
```bash
python3 validate_architecture.py
```

Expected: All 10 checks should pass

## Troubleshooting

### Action servers not found
- Make sure all action servers are running (check terminals 1-3)
- Verify with: `ros2 action list`

### Mission doesn't start
- Check that all three action servers are running first
- Mission control waits for all servers before starting

### Build errors
```bash
# Clean and rebuild
cd ~/your_workspace
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
```

## Package Structure

```
src/
├── charging_interfaces/     # Action definitions
│   ├── action/
│   │   ├── Navigate.action    # Navigation goals
│   │   ├── Manipulate.action  # Manipulation tasks
│   │   └── Detect.action      # Object detection
│
├── navigation/              # Navigation action server
├── manipulation/            # Manipulation action server  
├── perception/              # Perception action server
└── mission_control/         # State machine action client
```

## Key Features

- Asynchronous action-based communication
- Feedback monitoring during execution
- Automatic state transitions
- Mock implementations for testing without hardware
- No "g1_" prefixes in package names

## Next Steps

1. Replace mock action servers with real implementations
2. Integrate with actual hardware (Unitree SDK)
3. Add error recovery and retry logic
4. Implement safety checks and monitoring
5. Add visualization in RViz

## Documentation

- See `README.md` for detailed architecture information
- See `IMPLEMENTATION_SUMMARY.md` for complete implementation details
- See individual package source files for API documentation

## Support

For issues or questions, please refer to the project's GitHub repository.
