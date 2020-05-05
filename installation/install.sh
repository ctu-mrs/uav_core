#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "\"${last_command}\" command failed with exit code $?"' ERR

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

cd $MY_PATH

## | ----------------------- install ROS ---------------------- |

bash $MY_PATH/dependencies/ros.sh

## | -------------- install general dependencies -------------- |

bash $MY_PATH/dependencies/general.sh

## | --------------------- install gitman --------------------- |

bash $MY_PATH/dependencies/gitman.sh

gitman install

## | ---------------------- install tmux ---------------------- |

bash $MY_PATH/dependencies/tmux/install.sh

## | ------------------- install tmuxinator ------------------- |

bash $MY_PATH/dependencies/tmuxinator.sh

## | ----------------- install debugging tools ---------------- |

bash $MY_PATH/dependencies/gdb/install.sh

## | --------------------- install mavros --------------------- |

bash $MY_PATH/dependencies/mavros.sh --download

## | ------- add sourcing of shell additions to .bashrc ------- |

num=`cat ~/.bashrc | grep "shell_additions.sh" | wc -l`
if [ "$num" -lt "1" ]; then

  TEMP=`( cd "$MY_PATH/../miscellaneous/shell_additions" && pwd )`

  echo "Adding source to .bashrc"
  # set bashrc
  echo "
# source uav_core shell additions
source $TEMP/shell_additions.sh" >> ~/.bashrc

fi

## | ------ add uav_core environment variables to .bashrc ----- |

bash $MY_PATH/../miscellaneous/scripts/setup_rc_variables.sh

exit 0
