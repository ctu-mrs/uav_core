#!/bin/bash

device=$(lsusb | grep 6666:6666)

is_connected=false
was_connected=false

if [ -f "/home/mrs/test_dir/mem" ]; then
  was_connected=true
else
  was_connected=false
fi

if [ -z "${device}" ]
then
  is_connected=false
else
  is_connected=true
fi

if [ "$is_connected" = true ] ; then
  if [ "$was_connected" = true ] ; then
    echo "was connected" >> /home/mrs/test_dir/status.txt
  else
    > /home/mrs/test_dir/status.txt
    echo "Switching wifi NOW!" >> ~/test_dir/status.txt
    indoor=$(cat /etc/netplan/01-netcfg.yaml | grep indoor)
    if [ -z "${indoor}" ]
    then
      sed -i 's/mrs_ctu/mrs_ctu_indoor/g' /etc/netplan/01-netcfg.yaml
    else
      echo "in if" >> /home/mrs/test_dir/status.txt
      sed -i 's/mrs_ctu_indoor/mrs_ctu/g' /etc/netplan/01-netcfg.yaml
    fi
    echo "switched" >> /home/mrs/test_dir/status.txt
    touch /home/mrs/test_dir/mem
  fi
else
  echo "is not connected" >> /home/mrs/test_dir/status.txt
  rm /home/mrs/test_dir/mem
fi
