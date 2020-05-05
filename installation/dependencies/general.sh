#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "\"${last_command}\" command failed with exit code $?"' ERR

echo "$0: installing general dependencies"

sudo apt-get -y install cmake build-essential autotools-dev automake autoconf

# utilities
sudo apt -y install wget zip

# ros-related
sudo apt-get -y install \
  ros-melodic-multimaster-*\
  ros-melodic-visualization-msgs\
  ros-melodic-smach-*\
  ros-melodic-diagnostic-updater\
  ros-melodic-nav-msgs\
  ros-melodic-angles\
  ros-melodic-rosconsole-bridge\
  ros-melodic-gazebo-*\
  ros-melodic-eigen-conversions\
  ros-melodic-roslint\
  ros-melodic-xacro\
  ros-melodic-camera-info-manager\
  ros-melodic-control-toolbox\
  ros-melodic-image-view\
  ros-melodic-image-transport\
  ros-melodic-image-transport-plugins\
  ros-melodic-compressed-image-transport\
  ros-melodic-theora-image-transport\
  ros-melodic-rosbash\
  ros-melodic-rqt*\
  ros-melodic-tf2-sensor-msgs\
  ros-melodic-tf2-geometry-msgs\
  ros-melodic-octomap-msgs\
  ros-melodic-flexbe-behavior-engine\
  ros-melodic-joy\
  ros-melodic-hector-gazebo-plugins\
  ros-melodic-rosdoc-lite\
  ros-melodic-teraranger\
  ros-melodic-geographic-msgs\
  ros-melodic-tf2-eigen\
  ros-melodic-rviz-visual-tools\
  ros-melodic-catch-ros\
  ros-melodic-octomap\
  ros-melodic-sophus\
  ros-melodic-cmake-modules\

# python stuff
sudo apt-get -y python-prettytable python-argparse git-core python-empy python-serial python-bloom python-catkin-tools python-pip python3-pip python-future python3-future python-crcmod

# other
sudo apt -y install genromfs\
  ant\
  protobuf-compiler\
  libeigen3-dev\
  libopencv-dev\
  openocd\
  flex\
  bison\
  libncurses5-dev\
  libftdi-dev\
  libtool\
  zlib1g-dev\
  gcc-arm-none-eabi\
  libevent-dev\
  libncurses5-dev\
  expect-dev\
  moreutils\
  xvfb\
  ros-melodic-plotjuggler\
  libeigen3-dev\
  libsuitesparse-dev\
  protobuf-compiler\
  libnlopt-dev\
  distcc\
  ocl-icd-opencl-dev\
  ocl-icd-dev\
  ocl-icd-libopencl1\
  clinfo\
  opencl-headers\
  libgeographic-dev\
  geographiclib-tools\
  libx264-dev\
  libzstd-dev\
  libqcustomplot-dev\
  xutils-dev\
