#!/bin/bash

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

echo "Installing termviz"

sudo apt-get -y install cargo


export ROS_WORKSPACES_STORE=$ROS_WORKSPACES
export ROS_WORKSPACES=""
echo "Re-sourcing ROS setup"
source /opt/ros/${ROS_DISTRO}/setup.bash
cd $MY_PATH/../../utils/termviz
echo "Running cargo build on termviz"
cargo build --release
touch ./CATKIN_IGNORE
export ROS_WORKSPACES=$ROS_WORKSPACES_STORE
# source $HOME/mrs_workspace/devel/setup.bash

sudo ln -s $MY_PATH/../../utils/termviz/target/release/termviz /usr/local/bin/
