#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "\"${last_command}\" command failed with exit code $?"' ERR

unattended=0
subinstall_params=""
for param in "$@"
do
  echo $param
  if [ $param="--unattended" ]; then
    echo "installing in unattended mode"
    unattended=1
    subinstall_params="--unattended"
  fi
done

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

cd $MY_PATH

## | ----------------------- install ROS ---------------------- |

bash $MY_PATH/dependencies/ros.sh $subinstall_params

## | -------------- install general dependencies -------------- |

bash $MY_PATH/dependencies/gitman.sh $subinstall_params

## | --------------------- install gitman --------------------- |

bash $MY_PATH/dependencies/gitman.sh $subinstall_params

## | ---------------------- install tmux ---------------------- |

bash $MY_PATH/tmux/install.sh $subinstall_params

## | ------------------- install tmuxinator ------------------- |

bash $MY_PATH/dependencies/tmuxinator/install.sh $subinstall_params

## | ----------------- install debugging tools ---------------- |

bash $MY_PATH/gdb/install.sh $subinstall_params

## | ------------------- install submodules ------------------- |

gitman install --force

## | --------------------- install mavros --------------------- |

bash $MY_PATH/dependencies/mavros.sh --download $subinstall_params

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
