#!/bin/bash
### BEGIN INIT INFO
# Provides: tmux
# Required-Start:    $local_fs $network dbus
# Required-Stop:     $local_fs $network
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: start the uav
### END INIT INFO
if [ "$(id -u)" == "0" ]; then
  exec sudo -u mrs "$0" "$@"
fi

source $HOME/.bashrc

PROJECT_NAME=just_flying

MAIN_DIR=~/"bag_files"

# following commands will be executed first, in each window
pre_input="export ATHAME_ENABLED=0; mkdir -p $MAIN_DIR/$PROJECT_NAME"

# define commands
# 'name' 'command'
input=(
  'Rosbag' 'waitForRos; rosrun mrs_general record.sh'
  'Sensors' 'waitForRos; roslaunch mrs_general sensors.launch
'
  'Fire' 'waitForRos; roslaunch fire_detect three_thermo_cams.launch
'
  'SetRealsens' 'waitForRos; for (( ; ; )); do rosservice call '"$UAV_NAME"'/fire_detect/fire_localize/set_distance_to_realsense true; sleep 5; done
'
  'Hector' 'waitForRos; roslaunch hector_mapping uav.launch
'
  'MrsStatus' 'waitForRos; roslaunch mrs_status status.launch
'
  'Control' 'waitForRos; roslaunch mrs_general core.launch
'
  'MotorsOn' 'rosservice call /'"$UAV_NAME"'/control_manager/motors 1'
  'Takeoff' 'rosservice call /'"$UAV_NAME"'/uav_manager/takeoff'
  'ChangeEstimator' 'waitForOdometry; rosservice call /'"$UAV_NAME"'/odometry/change_estimator_type_string T265'
  'GoTo_FCU' 'rosservice call /'"$UAV_NAME"'/control_manager/goto_fcu "goal: [0.0, 0.0, 0.0, 0.0]"'
  'GoToRelative' 'rosservice call /'"$UAV_NAME"'/control_manager/goto_relative "goal: [0.0, 0.0, 0.0, 0.0]"'
  'Land' 'rosservice call /'"$UAV_NAME"'/uav_manager/land'
  'LandHome' 'rosservice call /'"$UAV_NAME"'/uav_manager/land_home'
  'E_hover' 'rosservice call /'"$UAV_NAME"'/control_manager/ehover'
  'Show_odom' 'waitForRos; rostopic echo /'"$UAV_NAME"'/odometry/slow_odom
'
  'Show_diag' 'waitForRos; rostopic echo /'"$UAV_NAME"'/odometry/diagnostics
'
  'Mav_diag' 'waitForRos; rostopic echo /'"$UAV_NAME"'/mavros_interface/diagnostics
'
  'KernelLog' 'tail -f /var/log/kern.log -n 100
'
  'roscore' 'roscore
'
  'KILL_ALL' 'dmesg; tmux kill-session -t '
)

init_window="MrsStatus"

###########################
### DO NOT MODIFY BELOW ###
###########################

SESSION_NAME=mav

# Absolute path to this script. /home/user/bin/foo.sh
SCRIPT=$(readlink -f $0)
# Absolute path this script is in. /home/user/bin
SCRIPTPATH=`dirname $SCRIPT`

if [ -z ${TMUX} ];
then
  TMUX= /usr/bin/tmux new-session -s "$SESSION_NAME" -d
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
  ITERATOR="1"
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
  /usr/bin/tmux new-window -t $SESSION_NAME:$(($i+1)) -n "${names[$i]}"
done

sleep 3

# start loggers
for ((i=0; i < ${#names[*]}; i++));
do
  /usr/bin/tmux pipe-pane -t $SESSION_NAME:$(($i+1)) -o "ts | cat >> $TMUX_DIR/$(($i+1))_${names[$i]}.log"
done

# send commands
for ((i=0; i < ${#cmds[*]}; i++));
do
  tmux send-keys -t $SESSION_NAME:$(($i+1)) "cd $SCRIPTPATH;${pre_input};${cmds[$i]}"
done

# identify the index of the init window
init_index=0
for ((i=0; i < ((${#names[*]})); i++));
do
  if [ ${names[$i]} == "$init_window" ]; then
    init_index=$(expr $i + 1)
  fi
done

/usr/bin/tmux select-window -t $SESSION_NAME:$init_index

/usr/bin/tmux -2 attach-session -t $SESSION_NAME

clear
