#!/bin/bash

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

sudo apt-get -y install cargo


export ROS_WORKSPACES_STORE=$ROS_WORKSPACES
export ROS_WORKSPACES=""
echo "A"
source /opt/ros/*/setup.bash
echo "B"
cd $MY_PATH/../../utils/termviz
cargo build --release
export ROS_WORKSPACES=$ROS_WORKSPACES_STORE
# source $HOME/mrs_workspace/devel/setup.bash

sudo ln -s $MY_PATH/../../utils/termviz/target/release/termviz /usr/local/bin/
