#!/bin/bash

MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

cd $MY_PATH

# Get the broadcast ip address
echo "Determining broadcast IP address"
if var1=$(ifconfig | grep -A1 wlan0:); then

  var2=$(echo $var1 |sed -n -e 's/^.*broadcast //p')

  if [ -z "$var2" ]; then
    success=false
    echo "Could not determine the broadcast IP address"
  else
    success=true
    echo "Broadcast address:"
    echo "$var2"
  fi

else
  success=false
  echo "Could not find the interface wlan0"
fi

echo "Setting BROADCAST_IP variable in .bashrc"

./get_set_rc_variable.sh "$HOME/.bashrc" "BROADCAST_IP" "$var2" "The broadcast IP address used for Nimbro network transport"

echo "BROADCAST_IP set"
