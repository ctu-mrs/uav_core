#!/bin/bash
# author: Tomas Baca

# get the path to this script
APP_PATH=`dirname "$0"`
APP_PATH=`( cd "$APP_PATH" && pwd )`

MY_WORKSPACE=workspace

# create the folder structure
mkdir -p ~/$MY_WORKSPACE/src

cd ~/$MY_WORKSPACE
catkin init
catkin config --extend ~/mrs_workspace/devel

# set build profiles
catkin config --profile default --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
catkin config --profile release --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
catkin profile set default

# clone templates repository
cd src
git clone --recursive git@mrs.felk.cvut.cz:uav/templates
cd ~/$MY_WORKSPACE
catkin build
