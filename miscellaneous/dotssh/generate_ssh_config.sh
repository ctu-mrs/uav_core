#!/bin/bash

key_name="~/.ssh/id_rsa_drone_login"

input=(
  'red'         'uav20'  '192.168.69.120'
  'green'       'uav21'  '192.168.69.121'
  'blue'        'uav22'  '192.168.69.122'
  'yellow'      'uav23'  '192.168.69.123'
  'pink'        'uav24'  '192.168.69.124'
  'black'       'uav25'  '192.168.69.125'
  'white'       'uav26'  '192.168.69.126'

  # our laptops, remove if needed

  'klaxalk' 'klaxalk-local' '192.168.69.11'
  'dan'     'dan-local'     '192.168.69.42'
)

# get path to script
SCRIPT_PATH="$( cd "$(dirname "$0")" ; pwd -P )"

if [ -x "$(whereis nvim | awk '{print $2}')" ]; then
  VIM_BIN="$(whereis nvim | awk '{print $2}')"
  HEADLESS="--headless"
elif [ -x "$(whereis vim | awk '{print $2}')" ]; then
  VIM_BIN="$(whereis vim | awk '{print $2}')"
  HEADLESS=""
fi

for ((i=0; i < ${#input[*]}; i++));
do
  ((i%3==0)) && nato[$i/3]="${input[$i]}"
  ((i%3==1)) && hostname[$i/3]="${input[$i]}"
  ((i%3==2)) && ip[$i/3]="${input[$i]}"
done

hostname=( "${hostname[@]}" "${nato[@]}" )
ip=( "${ip[@]}" "${ip[@]}" )

my_hostname=$( cat /etc/hostname )

for ((i=0; i < ${#hostname[*]}; i++)); do

  echo ""
  echo "Proceesing ${hostname[i]}"

  num=`cat ~/.ssh/config | grep "host ${hostname[i]}" | wc -l`
  if [ "$num" -lt "1" ]; then

    echo Creating new entry in .ssh/config for ${hostname[i]} ${ip[i]}

  else

    echo Updating entry in .ssh/config for ${hostname[i]} ${ip[i]}

    $VIM_BIN $HEADLESS -Ens -c "set ignorecase" -c "%g/.*host ${hostname[i]}.*/norm dap" -c "wqa" -- "$HOME/.ssh/config"

  fi

  echo "host ${hostname[i]}
  hostname ${ip[i]}
  user mrs
  identityfile $key_name
  " >> ~/.ssh/config

  # move the current entry to the bottom of the file
  $VIM_BIN $HEADLESS -Ens -c "set ignorecase" -c "%g/${ip[i]}\s.*/norm dapGp" -c "wqa" -- "$HOME/.ssh/config"

  num=`cat /etc/hosts | grep ".* ${hostname[i]}$" | wc -l`
  if [ "$num" -lt "1" ]; then

    echo Creating new entry in /etc/hosts for ${hostname[i]} ${ip[i]}

  else

    # delete the old entry
    echo "deleting old entry in /etc/hosts"
    sudo $VIM_BIN $HEADLESS -Ens -c "set ignorecase" -c "%g/.*\s${hostname[i]}$/norm dd" -c "wqa" -- "/etc/hosts"

  fi

  # only do it if its another uav
  # this is neccessary for NimbroNetwork
  if [[ "$my_hostname" != "${hostname[i]}" ]]; then

    echo Updating entry in /etc/hosts for ${hostname[i]} ${ip[i]}

    sudo bash -c "echo ${ip[i]} ${hostname[i]} >> /etc/hosts"

    sudo $VIM_BIN $HEADLESS -Ens -c "set ignorecase" -c "%g/${ip[i]}\s/norm ddGp" -c "wqa" -- "/etc/hosts"
    sudo $VIM_BIN $HEADLESS -Ens -c "set ignorecase" -c "normal Go" -c "wqa" -- "/etc/hosts"

  fi

done

sudo $VIM_BIN $HEADLESS -Ens -c "set ignorecase" -c "%g/^\\n$\\n$\\n/norm dd" -c "wqa" -- "/etc/hosts"
