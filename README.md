# Motor Control SDK
This repository contains a lightweight SDK for CAN communication with CubeMars AK-series motors and provides a high-level ROS2 interface.

## 🚧 Warning 🚧
This repository is a work in progress and is not currently stable.

## How To Use
This project now uses the standard ROS2 build system with `colcon`. It requires ROS2 Jazzy and a modern C++ compiler that supports C++17.

### Prerequisites
- ROS2 Jazzy installation
- `colcon` build tools
- C++17 compatible compiler

### Building
```bash
# Create a ROS2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this repository
git clone <this-repo-url> motor_control_sdk

# Build the package
cd ~/ros2_ws
colcon build --packages-select motor_control_sdk

# Source the workspace
source install/setup.bash
```

### Running
```bash
# Launch the motor driver node
ros2 launch motor_control_sdk motor_control.launch.py

# Or run the node directly
ros2 run motor_control_sdk motor_driver_node
```

### Usage as a Dependency
Add this to your `package.xml`:
```xml
<depend>motor_control_sdk</depend>
```

Add this to your `CMakeLists.txt`:
```cmake
find_package(motor_control_sdk REQUIRED)
ament_target_dependencies(your_target motor_control_sdk)
```
