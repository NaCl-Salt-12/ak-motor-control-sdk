# Motor Control SDK
This repository contains a lightweight SDK for CAN communication with CubeMars AK-series motors and provides a high-level ROS2 interface.

## 🚧 Warning 🚧
This repository is a work in progress and is not currently stable.

## How To Use
This project uses Bazel and requires a modern C++ compiler that supports C++23. Bazel builds ROS2 in a self-contained way, so you do not need to have ROS2 installed on your system. You only need to use the `bazel build` and `bazel run` commands.

Add the following to your `MODULE.bazel` file:
```python
bazel_dep(name = "motor-control-sdk")
git_override(
    module_name = "motor-control-sdk",
    remote = "git@github.com:Optimal-Robotics-Lab/motor-control-sdk.git",
    commit = "[insert the latest commit hash]",
)
```
