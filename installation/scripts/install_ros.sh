#!/bin/bash
# author: Tomas Baca

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt -y update
sudo apt -y install ros-melodic-desktop-full
sudo rosdep init
sudo rosdep fix-permissions
rosdep update

#############################################                                           
# install dependencies for libgeographiclib                                             
#############################################                                           
act_path=`pwd`                    
cd /opt/ros/melodic/lib/mavros            
sudo ./install_geographiclib_datasets.sh
cd $act_path
