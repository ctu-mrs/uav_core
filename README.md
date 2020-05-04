# The core modules for UAV

### Status Ros Melodic
[![Build Status](https://travis-ci.com/ctu-mrs/uav_core.svg?branch=master)](https://travis-ci.com/ctu-mrs/uav_core)

## System requirements
Required OS is Ubuntu 18.04 LTS 64-bit or its flavours that can install ROS Melodic. Suggested variant of OS installation is dual boot instead of virtualization that can be slow and can not handle well the simulation GUI.

## For newcomers I suggest reading our [wiki](https://mrs.felk.cvut.cz/gitlab/uav/uav_core/wikis/home)
The wiki pages contain some basic informations for better understanding of the system.

## If you find some bug or struggle with installation please create an [issue](https://mrs.felk.cvut.cz/gitlab/uav/uav_core/issues)
By creating an issue on this page you can help us improve the instructions, the code and also help others with same problem.

## Setting up SSH key for GITLAB

To be able to clone, commit and push changes to repositories on [our git](https://mrs.felk.cvut.cz/gitlab) you ought to set up ssh keys. Generate new ssh key as
```bash
ssh-keygen -t rsa -b 4096 -C "your_email@example.com"
```

and add contents of a file **~/.ssh/id_rsa.pub** as a new key to your account at https://mrs.felk.cvut.cz/gitlab/profile/keys .

```bash
cat ~/.ssh/id_rsa.pub
```

## Installing the uav_core

Paste following code into your terminal and press **enter**
```bash
cd /tmp
echo 'mkdir -p ~/git
cd ~/git
sudo apt update
sudo apt upgrade
sudo apt-get -y install git expect
expect -c "
spawn git clone git@mrs.felk.cvut.cz:uav/uav_core.git
expect {
  \"no)?\" {
    send "yes\\n"
    interact }
}"
cd ~/git/uav_core/installation
git pull
./install.sh' > clone.sh && source clone.sh
```

## Installing Tomas's linux-setup

This repo install's Tomas's Linux environment.
**Beware!** This might alter you existing configuration of some Linux tools (Vim, Tmux, i3wm, ranger, ...).
Refer to its [README](https://github.com/klaxalk/linux-setup/blob/master/README.md), for more information.
Installation is *not* obligatory and the MRS system will work without it.

Paste following code into your terminal and press **enter**
```bash
cd /tmp
echo "mkdir -p ~/git
cd ~/git
sudo apt-get -y install git
git clone https://github.com/klaxalk/linux-setup.git
cd linux-setup
./install.sh" > run.sh && source run.sh
```

## Use template for your own repo

Paste following to terminal and press *enter*.
The commands will remove template's git remote server
```bash
cd ~/workspace/src/templates
git remote rm origin
rm -rf .git
```
You can reuse the template by renaming it to your own node name such as 'my_own_node' with command:
```bash
cd ~/workspace/src/
mv templates my_own_nodes
```
Then create your own repo here - https://mrs.felk.cvut.cz/gitlab/projects/new
Follow Command line instructions for "Git global setup" and "Existing folder or Git repository" where use the my_own_nodes as the mentioned existing_folder.

If you are taking part in the IEEE RAS Summer School, please use e.g. https://github.com/ or https://bitbucket.org/ if you want to create your own repository.

## Polishing your ~/.bashrc file

The **~/.bashrc** file, is the main "configuration hub" for your terminal session and for the ROS system aswell.
The automatic installation should fill it with the correct stuff, but it might be misorganized.
Please edit your **.bashrc** and make its **bottom end** look like the following code (the order of the lines is the important notion).
Also, I have added some additional comments here, to make more clear what is happening at each line.

```bash
# initializes ROS itself, should the first line of our code in .bashrc
source /opt/ros/melodic/setup.bash

# tells ROS, where to find your programs built with ROS
source ~/workspace/devel/setup.bash

# initializes gazebo simulator and our simulation repository
source /usr/share/gazebo/setup.sh
source ~/git/simulation/install/share/simulation/setup.sh

# tells gazebo, where it should look for our 3D models for simulation
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/git/simulation/gazebo_files/models

export NATO_NAME="" # lower-case name of the UAV frame {alpha, bravo, charlie, ...}
export RUN_TYPE="simulation" # {simulation, uav}
export UAV_TYPE="f550" # {f550, f450, t650, eagle, naki}
export PROPULSION_TYPE="default" # {default, new_esc, ...}
export ODOMETRY_TYPE="gps" # {gps, optflow, hector, vio, ...}
export INITIAL_DISTURBANCE_X="0.0" # [N], external disturbance in the body frame
export INITIAL_DISTURBANCE_Y="0.0" # [N], external disturbance in the body frame
export STANDALONE="false" # disables the core nodelete manager
export SWAP_GARMINS="false" # swap up/down garmins
export PIXGARM="false" # true if Garmin lidar is connected throught Pixhawk
export SENSORS="" # {garmin_down, garmin_up, rplidar, realsense_front, teraranger, bluefox_optflow, realsense_brick, bluefox_brick}
export WORLD_NAME="simulation" # e.g.: "simulation" <= mrs_general/config/world_simulation.yaml
export MRS_STATUS="readme" # {readme, dynamics, balloon, avoidance, control_error, gripper}
export LOGGER_DEBUG="false" # sets the ros console output level to debug

#####################################################################
# Following code provides settings for Tomas's linux-setup repository

# path to the git root
export GIT_PATH=~/git

# path to the all ros workspaces (for vim)
export ROS_WORKSPACES="~/mrs_workspace ~/workspace"

# want to run tmux automatically with new terminal?
export RUN_TMUX=true

# uav_core shell additions
source ~/git/uav_core/miscellaneous/shell_additions/shell_additions.sh

# sourcing tomas's linux setup
source ~/git/linux-setup/appconfig/bash/dotbashrc
```
