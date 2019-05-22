#!/bin/bash
# author: Tomas Baca

sudo usermod -a -G dialout $USER

# this should be installed when running the script from README...
# but if you run install.sh directly...
sudo apt-get -y install git expect


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


#exit 1
# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

ROS_WORKSPACE=mrs_workspace

cd $MY_PATH
git submodule update --init --recursive

#############################################
# Install mrs.felk.cvut.cz certificate and git large file storage (LFS)
#############################################
$MY_PATH/scripts/install_mrs_certificate.sh
$MY_PATH/scripts/install_git_lfs.sh

#############################################
# Install Tomas's Linux Setup
#############################################

default=y
while true ; do
  if [[ "$unattended" == "1" ]]
  then
    resp=$default
  else
    [[ -t 0 ]] && { read -t 10 -n 2 -p $'\e[1;32mInstall Linux setup? [y/n] (default: '"$default"$')\e[0m\n' resp || resp=$default ; } 
  fi
  response=`echo $resp | sed -r 's/(.*)$/\1=/'`
  
  if [[ $response =~ ^(y|Y)=$ ]]
  then

    #############################################
    # Clone Tomas's linux-setup repository
    #############################################

    cd ~/git
    git clone https://github.com/klaxalk/linux-setup

    cd ~/git/linux-setup
    bash install.sh $subinstall_params

    break
  elif [[ $response =~ ^(n|N)=$ ]]
  then
    break
  else
    echo " What? \"$resp\" is not a correct answer. Try y+Enter."
  fi
done

#############################################
# Install ROS?
#############################################

default=y
while true; do
  if [[ "$unattended" == "1" ]]
  then
    resp=$default
  else
    [[ -t 0 ]] && { read -t 10 -n 2 -p $'\e[1;32mInstall ROS? [y/n] (default: '"$default"$')\e[0m\n' resp || resp=$default ; }
  fi
  response=`echo $resp | sed -r 's/(.*)$/\1=/'`

  if [[ $response =~ ^(y|Y)=$ ]]
  then

    bash $MY_PATH/scripts/install_ros.sh
    bash $MY_PATH/scripts/install_dependencies.sh

    break
  elif [[ $response =~ ^(n|N)=$ ]]
  then
    break
  else
    echo " What? \"$resp\" is not a correct answer. Try y+Enter."
  fi
done

#############################################
# install mavlink
#############################################

default=y
while true; do
  if [[ "$unattended" == "1" ]]
  then
    resp=$default
  else
    [[ -t 0 ]] && { read -t 10 -n 2 -p $'\e[1;32mInstall mavlink? [y/n] (default: '"$default"$')\e[0m\n' resp || resp=$default ; }
  fi
  response=`echo $resp | sed -r 's/(.*)$/\1=/'`

  if [[ $response =~ ^(y|Y)=$ ]]
  then

    bash $MY_PATH/scripts/install_mavlink.sh

    break
  elif [[ $response =~ ^(n|N)=$ ]]
  then
    break
  else
    echo " What? \"$resp\" is not a correct answer. Try y+Enter."
  fi
done

#############################################
# Prepare ros workspace
#############################################

# create the directory for the workspace
mkdir -p ~/$ROS_WORKSPACE/src

cd ~/$ROS_WORKSPACE

catkin config --profile debug --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_CXX_FLAGS='-std=c++17 -march=native -fno-diagnostics-color'  -DCMAKE_C_FLAGS='-march=native -fno-diagnostics-color'
catkin config --profile release --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_CXX_FLAGS='-std=c++17 -march=native -fno-diagnostics-color'  -DCMAKE_C_FLAGS='-march=native -fno-diagnostics-color'
catkin config --profile reldeb --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_CXX_FLAGS='-std=c++17 -march=native -fno-diagnostics-color' -DCMAKE_C_FLAGS='-march=native -fno-diagnostics-color'
catkin profile set reldeb

# link uav_core repository to mrs_workspace
cd src
ln -s ~/git/uav_core

# clone uav_modules repository
default=y
while true; do
  if [[ "$unattended" == "1" ]]
  then
    resp=$default
  else
    [[ -t 0 ]] && { read -t 10 -n 2 -p $'\e[1;32mClone uav_modules repository? [y/n] (default: '"$default"$')\e[0m\n' resp || resp=$default ; }
  fi
  response=`echo $resp | sed -r 's/(.*)$/\1=/'`

  if [[ $response =~ ^(y|Y)=$ ]]
  then

    cd ~/git

    # clone the repo
    git clone --recursive git@mrs.felk.cvut.cz:uav/uav_modules.git

    cd ~/git/uav_modules
    git pull

    # update its submodules
    cd ~/git/uav_modules
    git submodule update --init --recursive

    # link it to mrs_workspace
    cd ~/$ROS_WORKSPACE/src
    ln -s ~/git/uav_modules

    # install bluefox
    cd uav_modules/ros_packages/bluefox2/install
    bash install.sh

    break
  elif [[ $response =~ ^(n|N)=$ ]]
  then
    break
  else
    echo " What? \"$resp\" is not a correct answer. Try y+Enter."
  fi
done

#############################################
# Compile the workspace for the first time
#############################################

cd ~/$ROS_WORKSPACE
source /opt/ros/melodic/setup.bash
catkin build

#############################################
# Set up student's ros workspace
#############################################

default=y
while true; do
  if [[ "$unattended" == "1" ]]
  then
    resp=$default
  else
    [[ -t 0 ]] && { read -t 10 -n 2 -p $'\e[1;32mSet up student\'s workspace? [y/n] (default: '"$default"$')\e[0m\n' resp || resp=$default ; }
  fi
  response=`echo $resp | sed -r 's/(.*)$/\1=/'`

  if [[ $response =~ ^(y|Y)=$ ]]
  then

    bash $MY_PATH/scripts/set_my_workspace.sh

    break
  elif [[ $response =~ ^(n|N)=$ ]]
  then
    break
  else
    echo " What? \"$resp\" is not a correct answer. Try y+Enter."
  fi
done

#############################################
# Add sourcing of ROS to bashrc
#############################################

# add sourcing of ROS
line="source /opt/ros/melodic/setup.bash"

num=`cat ~/.bashrc | grep "$line" | wc -l`
if [ "$num" -lt "1" ]; then

  echo "Adding $line to your .bashrc"

  # set bashrc
  echo "
$line" >> ~/.bashrc

fi

#############################################
# Add sourcing of workspace to bashrc
#############################################

# add sourcing of ros workspace
line="source ~/workspace/devel/setup.bash"

num=`cat ~/.bashrc | grep "$line" | wc -l`
if [ "$num" -lt "1" ]; then

  echo "Adding $line to your .bashrc"

  # set bashrc
echo "$line" >> ~/.bashrc

fi

#############################################
# Prepare simulation
#############################################

default=y
while true; do
  if [[ "$unattended" == "1" ]]
  then
    resp=$default
  else
    [[ -t 0 ]] && { read -t 10 -n 2 -p $'\e[1;32mInstall simulation? [y/n] (default: '"$default"$')\e[0m\n' resp || resp=$default ; }
  fi
  response=`echo $resp | sed -r 's/(.*)$/\1=/'`

  if [[ $response =~ ^(y|Y)=$ ]]
  then

    #############################################
    # Clone simulation repository
    #############################################

    cd ~/git
    # clone the repo
    git clone git@mrs.felk.cvut.cz:uav/simulation.git

    cd simulation/installation
    ./install.sh

    break
  elif [[ $response =~ ^(n|N)=$ ]]
  then
    break
  else
    echo " What? \"$resp\" is not a correct answer. Try y+Enter."
  fi
done

toilet All done

cd ~
source .bashrc
