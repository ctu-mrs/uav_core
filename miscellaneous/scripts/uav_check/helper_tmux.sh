#!/bin/bash
if [ "$(id -u)" == "0" ]; then
  exec sudo -u mrs "$0" "$@"
fi

source $HOME/.bashrc

# location for storing the bag files
# * do not change unless you know what you are doing

# the project name
# * is used to define folder name in ~/$MAIN_DIR
PROJECT_NAME=pixhawk_test

# the name of the TMUX session
# * can be used for attaching as 'tmux a -t <session name>'
SESSION_NAME=pixhawk_test

# following commands will be executed first in each window
# * do NOT put ; at the end

# define commands
# 'name' 'command'
# * DO NOT PUT SPACES IN THE NAMES
# * "new line" after the command    => the command will be called after start
# * NO "new line" after the command => the command will wait for user's <enter>
input=(
  'Mavros' 'waitForRos; roslaunch mrs_uav_general mavros_uav.launch
'
  'Python' 'waitForRos; python3 helper_checker.py; sleep 1; tmux kill-session
'
  'roscore' 'roscore
'
)

# the name of the window to focus after start
init_window="Python"

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

sleep 1

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
