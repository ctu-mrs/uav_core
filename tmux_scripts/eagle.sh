#!/bin/bash

PROJECT_NAME=just_flying

MAIN_DIR=~/"bag_files"

# following commands will be executed first, in each window
pre_input="export ATHAME_ENABLED=0; mkdir -p $MAIN_DIR/$PROJECT_NAME;"

# define commands
# 'name' 'command'
input=(
  'Rosbag' 'waitForRos; roslaunch mrs_general record.launch project_name:='"$PROJECT_NAME"'
  '
  'Sensors' 'waitForRos; roslaunch mrs_general sensors.launch
'
  'MRS_control' 'waitForRos; roslaunch mrs_uav_manager eagle.launch
'
  'ChangeMode' 'waitForOdometry; rosservice call /uav10/odometry/change_estimator_type_string '
  'SetConstraints' 'waitForOdometry; rosservice call /uav10/gain_manager/set_constraints  '
	'MotorsOn' 'rosservice call /'"$UAV_NAME"'/control_manager/motors 1'
	'Takeoff' 'rosservice call /'"$UAV_NAME"'/uav_manager/takeoff'
	'Headless' 'rosservice call /'"$UAV_NAME"'/control_manager/mpc_tracker/headless 1'
  'GoTo' 'rosservice call /'"$UAV_NAME"'/control_manager/goto "goal: [0.0, 0.0, 1.5, 1.9]"'
  'GoToRelative' 'rosservice call /'"$UAV_NAME"'/control_manager/goto_relative "goal: [0.0, 0.0, 0.0, 0.0]"'
  'GoTo_left' 'rosservice call /'"$UAV_NAME"'/control_manager/goto "goal: [5.0, 5.0, 1.5, 1.9]"'
  'GoTo_right' 'rosservice call /'"$UAV_NAME"'/control_manager/goto "goal: [-5.0, -5.0, 1.5, 1.9]"'
	'Land' 'rosservice call /'"$UAV_NAME"'/uav_manager/land'
	'LandHome' 'rosservice call /'"$UAV_NAME"'/uav_manager/land_home'
  'Hover' 'rosservice call /'"$UAV_NAME"'/control_manager/hover' 
  'E_hover' 'rosservice call /'"$UAV_NAME"'/control_manager/ehover' 
  'Show_odom' 'waitForRos; rostopic echo /'"$UAV_NAME"'/odometry/slow_odom
'
  'Show_diag' 'waitForRos; rostopic echo /'"$UAV_NAME"'/odometry/diagnostics
'
  'Mav_diag' 'waitForRos; rostopic echo /'"$UAV_NAME"'/mavros_interface/diagnostics
'
  'Orb_slam' 'waitForRos; roslaunch orb_slam uav.launch'
  'diag' 'waitForRos; rostopic echo /diagnostics
'
	'KernelLog' 'tail -f /var/log/kern.log -n 100
'
  'roscore' 'roscore
'
  'Multimaster' 'waitForRos; roslaunch mrs_multimaster server.launch'
	'KILL_ALL' 'dmesg; tmux kill-session -t '
)

###########################
### DO NOT MODIFY BELOW ###
###########################

SESSION_NAME=mav

if [ -z ${TMUX} ];
then
  TMUX= tmux new-session -s "$SESSION_NAME" -d
  echo "Starting new session."
else
  echo "Already in tmux, leave it first."
  exit
fi

# get the iterator
ITERATOR_FILE="$MAIN_DIR/$PROJECT_NAME"/iterator.txt
if [ -e "$ITERATOR_FILE" ]
then 
  ITERATOR=`cat "$ITERATOR_FILE"`
  ITERATOR=$(($ITERATOR+1))
else
  echo "iterator.txt does not exist, creating it"
  touch "$ITERATOR_FILE"
  ITERATOR="0"
fi
echo "$ITERATOR" > "$ITERATOR_FILE"   

# create file for logging terminals' output
LOG_DIR="$MAIN_DIR/$PROJECT_NAME/"
SUFFIX=$(date +"%Y_%m_%d_%H_%M_%S")
SUBLOG_DIR="$LOG_DIR/"$ITERATOR"_"$SUFFIX""
TMUX_DIR="$SUBLOG_DIR/tmux"
mkdir -p "$SUBLOG_DIR"
mkdir -p "$TMUX_DIR"

# link the "latest" folder to the recently created one
rm "$LOG_DIR/latest"
rm "$MAIN_DIR/latest"
ln -sf "$SUBLOG_DIR" "$LOG_DIR/latest"
ln -sf "$SUBLOG_DIR" "$MAIN_DIR/latest"

# create arrays of names and commands
for ((i=0; i < ${#input[*]}; i++));
do
  ((i%2==0)) && names[$i/2]="${input[$i]}" 
	((i%2==1)) && cmds[$i/2]="${input[$i]}"
done

# run tmux windows
for ((i=0; i < ${#names[*]}; i++));
do
	tmux new-window -t $SESSION_NAME:$(($i+1)) -n "${names[$i]}"
done

sleep 2

# start loggers
for ((i=0; i < ${#names[*]}; i++));
do
	tmux pipe-pane -t $SESSION_NAME:$(($i+1)) -o "ts | cat >> $TMUX_DIR/$(($i+1))_${names[$i]}.log"
done

sleep 2

# send commands
for ((i=0; i < ${#cmds[*]}; i++));
do
	tmux send-keys -t $SESSION_NAME:$(($i+1)) "${pre_input};${cmds[$i]}"
done

sleep 2

tmux select-window -t $SESSION_NAME:3
tmux -2 attach-session -t $SESSION_NAME

clear
