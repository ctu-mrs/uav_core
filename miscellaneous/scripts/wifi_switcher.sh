#!/bin/bash

device=$(lsusb | grep 6666:6666)

is_connected=false
was_connected=false

if [ -f "/tmp/mem_wifi" ]; then
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
    echo "pes"
  else
    indoor=$(cat /etc/netplan/01-netcfg.yaml | grep indoor)
    if [ -z "${indoor}" ]
    then
      sed -i 's/mrs_ctu/mrs_ctu_indoor/g' /etc/netplan/01-netcfg.yaml
      netplan apply
    else
      sed -i 's/mrs_ctu_indoor/mrs_ctu/g' /etc/netplan/01-netcfg.yaml
      netplan apply
    fi
    touch /tmp/mem_wifi
  fi
else
  rm /tmp/mem_wifi
fi
