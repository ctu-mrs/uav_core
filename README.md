# The MRS UAV system core

![](.fig/thumbnail.jpg)

| UAV core build status | [![Build Status](https://github.com/ctu-mrs/uav_core/workflows/Noetic/badge.svg)](https://github.com/ctu-mrs/uav_core/actions) |
|-----------------------|--------------------------------------------------------------------------------------------------------------------------------|

| ROS Package                                                                               | 20.04                                                                                                                                                                    |
|-------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [mrs_bumper](https://github.com/ctu-mrs/mrs_bumper)                                       | [![Build Status](https://github.com/ctu-mrs/mrs_bumper/workflows/Noetic/badge.svg)](https://github.com/ctu-mrs/mrs_bumper/actions)                                       |
| [mrs_lib](https://github.com/ctu-mrs/mrs_lib)                                             | [![Build Status](https://github.com/ctu-mrs/mrs_lib/workflows/Noetic/badge.svg)](https://github.com/ctu-mrs/mrs_lib/actions)                                             |
| [mrs_mavros_interface](https://github.com/ctu-mrs/mrs_mavros_interface)                   | [![Build Status](https://github.com/ctu-mrs/mrs_mavros_interface/workflows/Noetic/badge.svg)](https://github.com/ctu-mrs/mrs_mavros_interface/actions)                   |
| [mrs_msgs](https://github.com/ctu-mrs/mrs_msgs)                                           | [![Build Status](https://github.com/ctu-mrs/mrs_msgs/workflows/Noetic/badge.svg)](https://github.com/ctu-mrs/mrs_msgs/actions)                                           |
| [mrs_rviz_plugins](https://github.com/ctu-mrs/mrs_rviz_plugins)                           | [![Build Status](https://github.com/ctu-mrs/mrs_rviz_plugins/workflows/Noetic/badge.svg)](https://github.com/ctu-mrs/mrs_rviz_plugins/actions)                           |
| [mrs_uav_controllers](https://github.com/ctu-mrs/mrs_uav_controllers)                     | [![Build Status](https://github.com/ctu-mrs/mrs_uav_controllers/workflows/Noetic/badge.svg)](https://github.com/ctu-mrs/mrs_uav_controllers/actions)                     |
| [mrs_uav_general](https://github.com/ctu-mrs/mrs_uav_general)                             | [![Build Status](https://github.com/ctu-mrs/mrs_uav_general/workflows/Noetic/badge.svg)](https://github.com/ctu-mrs/mrs_uav_general/actions)                             |
| [mrs_uav_managers](https://github.com/ctu-mrs/mrs_uav_managers)                           | [![Build Status](https://github.com/ctu-mrs/mrs_uav_managers/workflows/Noetic/badge.svg)](https://github.com/ctu-mrs/mrs_uav_managers/actions)                           |
| [mrs_uav_odometry](https://github.com/ctu-mrs/mrs_uav_odometry)                           | [![Build Status](https://github.com/ctu-mrs/mrs_uav_odometry/workflows/Noetic/badge.svg)](https://github.com/ctu-mrs/mrs_uav_odometry/actions)                           |
| [mrs_uav_status](https://github.com/ctu-mrs/mrs_uav_status)                               | [![Build Status](https://github.com/ctu-mrs/mrs_uav_status/workflows/Noetic/badge.svg)](https://github.com/ctu-mrs/mrs_uav_status/actions)                               |
| [mrs_uav_testing](https://github.com/ctu-mrs/mrs_uav_testing)                             | [![Build Status](https://github.com/ctu-mrs/mrs_uav_testing/workflows/Noetic/badge.svg)](https://github.com/ctu-mrs/mrs_uav_testing/actions)                             |
| [mrs_uav_trackers](https://github.com/ctu-mrs/mrs_uav_trackers)                           | [![Build Status](https://github.com/ctu-mrs/mrs_uav_trackers/workflows/Noetic/badge.svg)](https://github.com/ctu-mrs/mrs_uav_trackers/actions)                           |
| [mrs_uav_trajectory_generation](https://github.com/ctu-mrs/mrs_uav_trajectory_generation) | [![Build Status](https://github.com/ctu-mrs/mrs_uav_trajectory_generation/workflows/Noetic/badge.svg)](https://github.com/ctu-mrs/mrs_uav_trajectory_generation/actions) |
| **Third-party**                                                                           |                                                                                                                                                                          |
| [nlopt_ros](https://github.com/ctu-mrs/nlopt_ros)                                         |                                                                                                                                                                          |
| [tf2](https://github.com/ctu-mrs/tf2)                                                     |                                                                                                                                                                          |

## System requirements

Required OS is Ubuntu 20.04 LTS 64-bit or their flavors that can install ROS Noetic.
The suggested variant of OS installation is dual boot instead of virtualization that can be slow and can not handle well the simulation GUI.
We use [Gitman](https://github.com/jacebrowning/gitman) to manage the repository **submodules**.
The repository are supposed to be compiled by [catkin tools](https://catkin-tools.readthedocs.io).

## Installing uav_core

The preferred way is to install the whole [MRS UAV system](https://github.com/ctu-mrs/mrs_uav_system).
However, if you prefer to install only the *uav_core*, you can do the following:

1. Clone the repository,
2. place it into your *catkin workspace* ([guide](https://ctu-mrs.github.io/docs/software/catkin/managing_workspaces/managing_workspaces.html)),
3. call ```installation/install.sh``` to install the dependencies,
4. build your workspace.

## Finishing your .bashrc

The `install.sh` script will add the following to your .bashrc:
```bash
source /opt/ros/noetic/setup.bash

# source uav_core shell additions
source $HOME/git/uav_core/miscellaneous/shell_additions/shell_additions.sh
export UAV_NAME="uav1"
export NATO_NAME="" # lower-case name of the UAV frame {alpha, bravo, charlie, ...}
export UAV_MASS="3.0" # [kg], used only with real UAV
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
```

Our shell additions
```source $HOME/git/uav_core/miscellaneous/shell_additions/shell_additions.sh```
provide some useful command line aliases (git, catkin, cd) and some neccessary shell functions (wait\* commands for simulation).

All the environment variables define a particular configuration of a UAV (through launch files).
The variables apply mostly to real hardware. They allow us to make the **core** universal without the need of hardcoding configurations for a particular UAV, but only for a UAV type.
In a simulation, they are mostly *overloaded* in each tmuxinator session.

## The `cd` alias

One of the features of our [shell_additions.sh](https://github.com/ctu-mrs/uav_core/blob/master/miscellaneous/shell_additions/shell_additions.sh) that might be intrusive is our `cd` alias.
Our `cd` is capable of automatically traversing from locations that have been symlinked somewhere, to the linked location.
This is especially useful if you (like us) clone git repositories under a specific location, e.g., `~/git/my_repository` and then link them to catkin workspaces, e.g., `~/project2_workspace/src/my_repository`.
When you call
```bash
cd ~/git/my_repository
```
our alias will put you in
```bash
user@machine:~/project2_workspace/src/my_repository$
```

A symlink map (stored in `/tmp/symlink_array.sh`) for all your workspaces is built automatically after boot (after the first start of the terminal) by a detached tmux session using the *silversearcher-ag* utility.
Only the workspaces in the environment variable
```bash
export ROS_WORKSPACES="~/mrs_workspace ~/modules_workspace ~/workspace ~/project2_workspace"
```
are considered.
The `ROS_WORKSPACES` variable is added during the installation of [MRS UAV system](https://github.com/ctu-mrs/mrs_uav_system) since that is where we set up the workspaces for you.
