#!/bin/bash

# ROS2 Jazzy Installation Script for Ubuntu 24.04 (Noble)
# This script installs ROS2 Jazzy and required build tools

set -e

echo "Installing ROS2 Jazzy..."

# Update system
sudo apt update

# Install required packages
sudo apt install -y software-properties-common curl

# Add ROS2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package list
sudo apt update

# Install ROS2 Jazzy Desktop
sudo apt install -y ros-jazzy-desktop

# Install colcon build tools
sudo apt install -y python3-colcon-common-extensions

# Install additional build dependencies
sudo apt install -y ros-jazzy-ament-cmake ros-jazzy-ament-cmake-python

# Install development tools
sudo apt install -y python3-pip python3-rosdep python3-vcstool

# Initialize rosdep
sudo rosdep init || true
rosdep update

echo "ROS2 Jazzy installation complete!"
echo ""
echo "To use ROS2, run:"
echo "source /opt/ros/jazzy/setup.bash"
echo ""
echo "Add this to your ~/.bashrc to source automatically:"
echo "echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc"