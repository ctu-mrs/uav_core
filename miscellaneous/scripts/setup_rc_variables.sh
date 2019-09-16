#!/bin/bash

MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

cd $MY_PATH

RCFILE=~/."$SNAME"rc

# variable name, value, commentary
input=(
  'UAV_NAME' 'uav1' ''
  'UAV_MASS' '3.0' '[kg]'
  'RUN_TYPE' 'simulation' '{simulation, uav}'
  'UAV_TYPE' 'f550' '{f550, f450, t650, ...}'
  'PROPULSION_TYPE' 'default' '{default, new_esc, ...}'
  'ODOMETRY_TYPE' 'gps' '{gps, optflow, hector, vio, ...}'
  'ODOMETRY_TYPE' 'gps' '{gps, optflow, hector, vio, ...}'
  'INITIAL_DISTURBANCE_X' '0.0' '[N]'
  'INITIAL_DISTURBANCE_Y' '0.0' '[N]'
  'STANDALONE' 'false' ''
  'SWAP_GARMINS' 'false' ''
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
  ./get_set_rc_variable.sh "${vars[$i]}" "${values[$i]}" "${comments[$i]}"
done
