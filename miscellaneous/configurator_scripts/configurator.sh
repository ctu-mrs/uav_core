#!/bin/bash
this_script_path="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
device=$(lsusb | grep -o ffff:ff..)
script_name=${this_script_path}"/autoscripts/"${device: -2}".sh"

is_connected=false
was_connected=false

if [ -f "/tmp/mem_conf" ]; then
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
  if [ "$was_connected" = false ] ; then
    eval $script_name
    touch /tmp/mem_conf
  fi
else
  rm /tmp/mem_conf
fi
