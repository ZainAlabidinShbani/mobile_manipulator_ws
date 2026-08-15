#!/usr/bin/env bash
# Environment Preflight Commands for Mobile Manipulator Project (ROS 2 Humble)

set -e

echo "=== 1. Updating APT & Installing ROS 2 Humble Dependencies ==="
source /opt/ros/humble/setup.bash

sudo apt update
sudo apt install -y \
  ros-humble-ur-description \
  ros-humble-ur-msgs \
  ros-humble-robotiq-description \
  ros-humble-realsense2-description \
  ros-humble-realsense2-camera \
  ros-humble-moveit \
  ros-humble-moveit-setup-assistant \
  ros-humble-nav2-bringup \
  ros-humble-navigation2 \
  ros-humble-slam-toolbox \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gz-ros2-control \
  ros-humble-ros-gz \
  ros-humble-xacro \
  ros-humble-joint-state-publisher-gui \
  ros-humble-tf2-tools \
  ros-humble-tf-transformations \
  python3-colcon-common-extensions \
  python3-rosdep

echo "=== 2. Installing Python Packages ==="
pip install ultralytics opencv-python transforms3d mss

echo "=== 3. Workspace Rosdep Preflight ==="
cd ~/mobile_manipulator_ws
rosdep update || true
rosdep install --from-paths src --ignore-src -r -y

echo "=== 4. Verifying ROS 2 Packages ==="
source /opt/ros/humble/setup.bash
ros2 pkg list | grep -E "ur_description|robotiq_description|realsense2_description|moveit|nav2"

echo "=== Preflight Complete ==="
