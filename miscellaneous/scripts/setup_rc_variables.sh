#!/bin/bash

MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

cd $MY_PATH

# variable name, value, commentary
input=(
  'UAV_NAME' 'uav1' ''
  'NATO_NAME' '' 'lower-case name of the UAV frame {alpha, bravo, charlie, ...}'
  'UAV_MASS' '3.0' '[kg], used only with real UAV'
  'RUN_TYPE' 'simulation' '{simulation, uav}'
  'UAV_TYPE' 'f550' '{f550, f450, t650, eagle, naki}'
  'PROPULSION_TYPE' 'default' '{default, new_esc, ...}'
  'ODOMETRY_TYPE' 'gps' '{gps, optflow, hector, vio, ...}'
  'INITIAL_DISTURBANCE_X' '0.0' '[N], external disturbance in the body frame'
  'INITIAL_DISTURBANCE_Y' '0.0' '[N], external disturbance in the body frame'
  'STANDALONE' 'false' 'disables the core nodelete manager'
  'SWAP_GARMINS' 'false' 'swap up/down garmins'
  'PIXGARM' 'false' 'true if Garmin lidar is connected throught Pixhawk'
  'SENSORS' '' '{garmin_down, garmin_up, rplidar, realsense_front, teraranger, bluefox_optflow, realsense_brick, bluefox_brick}'
  'WORLD_NAME' 'simulation' 'e.g.: "simulation" <= mrs_general/config/world_simulation.yaml'
  'MRS_STATUS' 'readme' '{readme, dynamics, balloon, avoidance, control_error, gripper}'
  'LOGGER_DEBUG' 'false' 'sets the ros console output level to debug'
)

# create arrays
for ((i=0; i < ${#input[*]}; i++));
do
  ((i%3==0)) && vars[$i/3]="${input[$i]}"
  ((i%3==1)) && values[$i/3]="${input[$i]}"
  ((i%3==2)) && comments[$i/3]="${input[$i]}"
done

for ((i=0; i < ((${#vars[*]})); i++));
do
  ./get_set_rc_variable.sh "$HOME/.bashrc" "${vars[$i]}" "${values[$i]}" "${comments[$i]}"

  if [ -e $HOME/.zshrc ]; then
    ./get_set_rc_variable.sh "$HOME/.zshrc" "${vars[$i]}" "${values[$i]}" "${comments[$i]}"
  fi
done
