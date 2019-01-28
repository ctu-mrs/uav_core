# The core modules for UAV

## System requirements
Required OS is Ubuntu 18.04 LTS or its flavours that can install ROS Melodic. Suggested variant of OS installation is dual boot instead of virtualization that can be slow and can not handle well the simulation GUI.

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

## Cloning and running automatic installation

Paste following code into your terminal and press **enter**
```bash
cd /tmp
echo 'mkdir -p ~/git
cd ~/git
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

# mass of a drone is set here (mainly for "the real world", where each drone has unique mass)
# the simulated drone's mass should be ~3.0 kg
export UAV_MASS=3.0

#####################################################################
# Following code provides settings for Tomas's linux-setup repository

# where should ctags look for sources to parse?
# -R dir1 -R dir2 ...
export CTAGS_SOURCE_DIR="-R ~/mrs_workspace -R ~/workspace"

# where should ctags look for sources to parse?
# CTAGS FROM THOSE FILE WILL BE CREATED ONLY ONCE
# -R dir1 -R dir2 ...
export CTAGS_ONCE_SOURCE_DIR="-R /opt/ros/melodic/include"

# path to the git root
export GIT_PATH=~/git

# path to the ros workspace
export ROS_WORKSPACE="~/mrs_workspace ~/workspace"

# want to run tmux automatically with new terminal?
export RUN_TMUX=true

# sourcing tomas's tmux preparation
source ~/git/linux-setup/appconfig/bash/dotbashrc
```
