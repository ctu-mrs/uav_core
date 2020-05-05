#!/bin/bash
# author: Tomas Baca

sudo apt-get -y install python-prettytable python-argparse git-core wget zip python-empy cmake build-essential genromfs ant protobuf-compiler libeigen3-dev libopencv-dev python-serial openocd flex bison libncurses5-dev autoconf texinfo build-essential libftdi-dev libtool zlib1g-dev python-empy gcc-arm-none-eabi ros-melodic-xacro ros-melodic-camera-info-manager ros-melodic-control-toolbox ros-melodic-image-view ros-melodic-image-transport ros-melodic-image-transport-plugins ros-melodic-compressed-image-transport ros-melodic-theora-image-transport autotools-dev automake libevent-dev libncurses5-dev expect-dev moreutils ros-melodic-multimaster-* ros-melodic-visualization-msgs ros-melodic-smach-* ros-melodic-diagnostic-updater ros-melodic-nav-msgs ros-melodic-angles ros-melodic-rosconsole-bridge ros-melodic-gazebo-* ros-melodic-eigen-conversions sl python-catkin-tools ros-melodic-roslint xvfb ros-melodic-rosbash ros-melodic-rqt* ros-melodic-tf2-sensor-msgs ros-melodic-tf2-geometry-msgs ros-melodic-octomap-msgs python-empy git tmux ros-melodic-plotjuggler libeigen3-dev libsuitesparse-dev protobuf-compiler libnlopt-dev ros-melodic-octomap ros-melodic-sophus ros-melodic-cmake-modules distcc ocl-icd-opencl-dev ocl-icd-dev ocl-icd-libopencl1 clinfo ros-melodic-rviz-visual-tools opencl-headers clinfo ros-melodic-geographic-msgs ros-melodic-tf2-eigen libgeographic-dev geographiclib-tools python-bloom python-pip python3-pip python-future python3-future ros-melodic-catch-ros libx264-dev libzstd-dev libqcustomplot-dev espeak ros-melodic-teraranger python-crcmod xutils-dev ruby gem ros-melodic-flexbe-behavior-engine ros-melodic-joy ros-melodic-hector-gazebo-plugins ros-melodic-rosdoc-lite

#remove unsupported mavros-msgs
sudo apt-get -y purge ros-melodic-mavros-msgs

sudo pip3 install gitman
sudo -H pip3 install gitman
