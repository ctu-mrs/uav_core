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

# location for storing the bag files
# * do not change unless you know what you are doing
MAIN_DIR=~/"bag_files"

# the project name
# * is used to define folder name in ~/$MAIN_DIR
PROJECT_NAME=rename_this

# the name of the TMUX session
# * can be used for attaching as 'tmux a -t <session name>'
SESSION_NAME=mav

# following commands will be executed first in each window
# * do NOT put ; at the end
pre_input="mkdir -p $MAIN_DIR/$PROJECT_NAME; export WORLD_FILE=./world.yaml"

# define commands
# 'name' 'command'
# * DO NOT PUT SPACES IN THE NAMES
# * "new line" after the command    => the command will be called after start
# * NO "new line" after the command => the command will wait for user's <enter>
input=(
  'Rosbag' 'waitForOffboard; ./record.sh
'
  'NodeChecker' 'waitForRos; roslaunch mrs_uav_general node_crash_checker.launch
'
  'Nimbro' 'waitForRos; rosrun mrs_uav_general run_nimbro.py custom_configs/nimbro.yaml custom_configs/uav_names.yaml
'
  'Sensors' 'waitForRos; roslaunch mrs_uav_general sensors.launch
'
  'Status' 'waitForRos; roslaunch mrs_uav_status status.launch
'
  'Control' 'waitForRos; roslaunch mrs_uav_general core.launch config_constraint_manager:=./custom_configs/constraint_manager.yaml config_control_manager:=./custom_configs/control_manager.yaml config_mpc_tracker:=./custom_configs/mpc_tracker.yaml config_odometry:=./custom_configs/odometry.yaml config_uav_manager:=./custom_configs/uav_manager.yaml config_uav_names:=./custom_configs/uav_names.yaml config_landoff_tracker:=./custom_configs/landoff_tracker.yaml
'
  'AutoStart' 'waitForRos; roslaunch mrs_uav_general automatic_start.launch custom_config:=./custom_configs/automatic_start.yaml
'
  'slow_odom' 'waitForRos; rostopic echo /'"$UAV_NAME"'/odometry/slow_odom
'
  'odom_diag' 'waitForRos; rostopic echo /'"$UAV_NAME"'/odometry/diagnostics
'
  'mavros_diag' 'waitForRos; rostopic echo /'"$UAV_NAME"'/mavros_interface/diagnostics
'
  'kernel_log' 'tail -f /var/log/kern.log -n 100
'
  'roscore' 'roscore
'
)

# the name of the window to focus after start
init_window="Status"

# automatically attach to the new session?
# {true, false}, default true
attach="true"

###########################
### DO NOT MODIFY BELOW ###
###########################

# prefere the user-compiled tmux
if [ -f /usr/local/bin/tmux ]; then
  export TMUX_BIN=/usr/local/bin/tmux
else
  export TMUX_BIN=/usr/bin/tmux
fi

# find the session
FOUND=$( $TMUX_BIN ls | grep $SESSION_NAME )

if [ $? == "0" ]; then

  echo "The session already exists"
  exit
fi

# Absolute path to this script. /home/user/bin/foo.sh
SCRIPT=$(readlink -f $0)
# Absolute path this script is in. /home/user/bin
SCRIPTPATH=`dirname $SCRIPT`

if [ -z ${TMUX} ];
then
  TMUX= $TMUX_BIN new-session -s "$SESSION_NAME" -d
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
  mkdir -p "$MAIN_DIR/$PROJECT_NAME"
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
rm "$LOG_DIR/latest" > /dev/null 2>&1
rm "$MAIN_DIR/latest" > /dev/null 2>&1
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
  $TMUX_BIN new-window -t $SESSION_NAME:$(($i+1)) -n "${names[$i]}"
done

sleep 3

# start loggers
for ((i=0; i < ${#names[*]}; i++));
do
  $TMUX_BIN pipe-pane -t $SESSION_NAME:$(($i+1)) -o "ts | cat >> $TMUX_DIR/$(($i+1))_${names[$i]}.log"
done

# send commands
for ((i=0; i < ${#cmds[*]}; i++));
do
  $TMUX_BIN send-keys -t $SESSION_NAME:$(($i+1)) "cd $SCRIPTPATH;${pre_input};${cmds[$i]}"
done

# identify the index of the init window
init_index=0
for ((i=0; i < ((${#names[*]})); i++));
do
  if [ ${names[$i]} == "$init_window" ]; then
    init_index=$(expr $i + 1)
  fi
done

$TMUX_BIN select-window -t $SESSION_NAME:$init_index

if [[ "$attach" == "true" ]]; then
  $TMUX_BIN -2 attach-session -t $SESSION_NAME
else
  echo "The session was started"
  echo "You can later attach by calling:"
  echo "  tmux a -t $SESSION_NAME"
fi
