#!/bin/bash

SESSION_NAME=uav1
UAV_NAME=$SESSION_NAME

# following commands will be executed first, in each window
pre_input="export UAV_NAME=$UAV_NAME; export ATHAME_ENABLED=0"

# define commands
# 'name' 'command'
input=(
  'Roscore' 'roscore
'
  'Gazebo' "waitForRos; roslaunch simulation simulation_mbzirc_landing.launch gui:=true
"
  # 'Spawn' "waitForSimulation; spawn 1 --run --delete --enable-rangefinder --enable-ground-truth --enable-realsense-top --enable-bluefox-camera --file ~/mrs_workspace/src/uav_core/ros_packages/mrs_odometry/config/init_pose/init_pose.csv
  'Spawn' "waitForSimulation; spawn 1 --run --delete --enable-rangefinder --enable-ground-truth --enable-bluefox-camera --file ~/mrs_workspace/src/uav_core/ros_packages/mrs_odometry/config/init_pose/init_pose.csv
"
  'MRS_control' "waitForOdometry; roslaunch mrs_mav_manager simulation.launch
"
  'LKF' "waitForRos; rostopic echo /uav1/odometry/lkf_states_out
"
  'Diag' "waitForRos; rostopic echo /uav1/odometry/diag
"
  'ChangeEst' 'rosservice call /uav1/odometry/change_estimator_type_string gps'
  'OpticFlow' "waitForRos; roslaunch optic_flow simulation.launch gui:=false
"
  "PrepareUAV" "waitForControl; rosservice call /$UAV_NAME/mavros/cmd/arming 1; rosservice call /$UAV_NAME/control_manager/motors 1; rosservice call /$UAV_NAME/mavros/set_mode 0 offboard; rosservice call /$UAV_NAME/mav_manager/takeoff;
"
  'Camera_follow' "waitForOdometry; gz camera -c gzclient_camera -f uav1
"
  'GoTo' "rosservice call /$UAV_NAME/control_manager/goto \"goal: [20.0, 0.0, 1.5, 0.0]\""
  'GoToRelative' "rosservice call /$UAV_NAME/control_manager/goto_relative \"goal: [0.0, 0.0, 0.0, 0.0]\""
  'RVIZ' "waitForOdometry; nice -n 15 rosrun rviz rviz -d ~/mrs_workspace/src/uav_core/ros_packages/mrs_odometry/rviz/uav1_odometry_single.rviz
"
)


if [ -z ${TMUX} ];
then
  TMUX= tmux new-session -s $SESSION_NAME -d
  echo "Starting new session."
else
  SESSION_NAME="$(tmux display-message -p '#S')"
fi

# create arrays of names and commands
for ((i=0; i < ${#input[*]}; i++));
do
  ((i%2==0)) && names[$i/2]="${input[$i]}" 
	((i%2==1)) && cmds[$i/2]="${input[$i]}"
done

# run tmux windows
for ((i=0; i < ${#names[*]}; i++));
do
	tmux new-window -t $SESSION_NAME:$(($i+10)) -n "${names[$i]}"
done

sleep 4

# send commands
for ((i=0; i < ${#cmds[*]}; i++));
do
	tmux send-keys -t $SESSION_NAME:$(($i+10)) "${pre_input};
${cmds[$i]}"
done

sleep 4

tmux select-window -t $SESSION_NAME:0
tmux -2 attach-session -t $SESSION_NAME

clear
