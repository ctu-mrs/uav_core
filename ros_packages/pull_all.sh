#!/bin/bash

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

cd $MY_PATH

input=(
"mrs_bumper" "master"
"mrs_lib" "master"
"mrs_mavros_interface" "master"
"mrs_msgs" "master"
"mrs_optic_flow" "master"
"mrs_rviz_plugins" "master"
"mrs_uav_controllers" "master"
"mrs_uav_general" "master"
"mrs_uav_managers" "master"
"mrs_uav_odometry" "master"
"mrs_uav_status" "master"
"mrs_uav_testing" "master"
"mrs_uav_trackers" "master"
)

# create arrays of names and commands
for ((i=0; i < ${#input[*]}; i++));
do
  ((i%2==0)) && repos[$i/2]="${input[$i]}"
  ((i%2==1)) && branches[$i/2]="${input[$i]}"
done

# copy the files
for ((i=0; i < ((${#repos[*]})); i++));
do

  echo Pulling "${repos[$i]}", branch ${branches[$i]}

  cd ${repos[$i]}

  git checkout ${branches[$i]}
  git pull

  cd ..

  echo ""

done
