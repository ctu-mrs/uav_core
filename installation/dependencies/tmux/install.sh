#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

echo "$0: uninstalling pre-installed tmux"
sudo apt -y remove tmux

echo "$0: installing tmux build dependencies"
sudo apt -y install libevent-dev bison

echo "$0: building tmux"

# compile and install custom tmux
cd $MY_PATH/../../../utils/tmux
( ./autogen.sh && ./configure && make && sudo make install-binPROGRAMS ) || ( echo "Tmux compilation failed, installing normal tmux" && sudo apt -y install tmux)
git clean -fd

FILE=$HOME/.tmux.conf
if [ -e "$FILE" ]; then
  echo "$0: .tmux.conf exists, not copying"
else
  echo "$0: copying .tmux.conf"
  ln -sf $MY_PATH/dottmux.conf $FILE
fi
