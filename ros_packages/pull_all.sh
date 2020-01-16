#!/bin/bash

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

cd $MY_PATH

dirs=(
"mrs_controllers"
"mrs_general"
"mrs_lib"
"mrs_mavros_interface"
"mrs_msgs"
"mrs_odometry"
"mrs_optic_flow"
"mrs_status"
"mrs_trackers"
"mrs_uav_manager"
)

# copy the files
for ((i=0; i < ${#dirs[*]}; i++));
do

  echo Pulling ${dirs[$i]}

  cd ${dirs[$i]}

  git pull

  cd ..

  echo ""

done
