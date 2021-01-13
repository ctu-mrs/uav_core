#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

echo "$0: Running ros1_bridge, make sure you have ROS1 core already running"

source ~/mrs_workspace/devel/setup.bash
source ~/ros2_bridge_workpsace/install/setup.bash

ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
