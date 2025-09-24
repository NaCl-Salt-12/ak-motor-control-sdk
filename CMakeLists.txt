cmake_minimum_required(VERSION 3.8)
project(motor_control_sdk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Use C++17 as it's more widely supported than C++23 for ROS2 Jazzy
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCommand.msg"
  "msg/MotorCommands.msg"
  "msg/MotorState.msg"
  "msg/MotorStates.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

# Include directories
include_directories(
  motor_control_sdk/lib
  motor_control_sdk/lib/driver
  motor_control_sdk/lib/communication
  motor_control_sdk/lib/communication/protocols
)

# Motor specs library
add_library(motor_specs
  motor_control_sdk/lib/motor_specs.h
)
set_target_properties(motor_specs PROPERTIES LINKER_LANGUAGE CXX)

# Containers library
add_library(containers
  motor_control_sdk/lib/driver/containers.h
)
set_target_properties(containers PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(containers motor_specs)

# CAN Bus library
add_library(can_bus
  motor_control_sdk/lib/communication/can_bus.cc
  motor_control_sdk/lib/communication/can_bus.h
)
target_link_libraries(can_bus)

# Motor Reply Protocol library
add_library(motor_reply
  motor_control_sdk/lib/communication/protocols/motor_reply.h
)
set_target_properties(motor_reply PROPERTIES LINKER_LANGUAGE CXX)

# Force Control Protocol library
add_library(force_control_protocol
  motor_control_sdk/lib/communication/protocols/force_control_protocol.h
)
set_target_properties(force_control_protocol PROPERTIES LINKER_LANGUAGE CXX)

# Motor Driver library
add_library(motor_driver
  motor_control_sdk/lib/driver/motor_driver.cc
  motor_control_sdk/lib/driver/motor_driver.h
)
ament_target_dependencies(motor_driver rclcpp)
target_link_libraries(motor_driver
  containers
  can_bus
  motor_reply
  force_control_protocol
)

# Link the generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(motor_driver "${cpp_typesupport_target}")

# Install libraries
install(TARGETS
  motor_specs
  containers
  can_bus
  motor_reply
  force_control_protocol
  motor_driver
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY motor_control_sdk/lib/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

# Sample executable (you can create more complex nodes as needed)
add_executable(motor_driver_node
  src/motor_driver_node.cpp
)
ament_target_dependencies(motor_driver_node rclcpp)
target_link_libraries(motor_driver_node motor_driver)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(motor_driver_node "${cpp_typesupport_target}")

# Install executable
install(TARGETS motor_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_export_libraries(motor_driver)

ament_package()