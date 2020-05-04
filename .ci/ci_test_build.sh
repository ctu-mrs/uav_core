#!/bin/bash
# author: Robert Penicka

echo "Starting test build" 
cd ~/catkin_ws
source /opt/ros/melodic/setup.bash
catkin build
echo "Ended test build"