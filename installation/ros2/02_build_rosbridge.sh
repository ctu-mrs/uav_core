#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

echo "$0: Building ros1_bridge"

ROS_BRIDGE_WS_PATH=~/ros2_bridge_workspace

# create ros2_workspace
[ ! -e $ROS_BRIDGE_WS_PATH/src ] && mkdir -p $ROS_BRIDGE_WS_PATH/src

clone_ros1_bridge() {
  cd $ROS_BRIDGE_WS_PATH/src
  [ ! -e ros1_bridge ] && git clone https://github.com/ros2/ros1_bridge
  cd ros1_bridge
  git fetch
  git checkout foxy
}

clone_mrs_msgs() {
  cd $ROS_BRIDGE_WS_PATH/src
  [ ! -e mrs_msgs ] && git clone https://github.com/ctu-mrs/mrs_msgs
  cd mrs_msgs
  git fetch
  git checkout ros2
}

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

clone_ros1_bridge
clone_mrs_msgs
build_mrs_msgs
build_ros1_bridge
