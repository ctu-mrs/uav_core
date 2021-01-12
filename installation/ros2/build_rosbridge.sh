#!/bin/bash

ROS_BRIDGE_WS_PATH=~/ros2_bridge_workpsace

# create ros2_workspace
cd
mkdir -p $ROS_BRIDGE_WS_PATH/src

# clone repositories
cd $ROS_BRIDGE_WS_PATH/src
git clone https://github.com/ros2/ros1_bridge
cd ros1_bridge
git fetch
git checkout foxy

cd $ROS_BRIDGE_WS_PATH/src
git clone https://github.com/ctu-mrs/mrs_msgs
cd mrs_msgs
git fetch
git checkout ros2

build_mrs_msgs() {
  cd $ROS_BRIDGE_WS_PATH
  source /opt/ros/foxy/setup.bash
  colcon build --symlink-install --packages-skip ros1_bridge
}

build_ros1_bridge() {
  cd $ROS_BRIDGE_WS_PATH
  source ~/mrs_workspace/devel/setup.bash
  source $ROS_BRIDGE_WS_PATH/install/setup.bash
  colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
}

build_mrs_msgs
build_ros1_bridge
