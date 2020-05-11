#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

options=$(getopt -l "download,compile,dryrun" -o "" -a -- "$@")

eval set -- "$options"

while true
do
  case $1 in
    --download)
      DOWNLOAD=true
      shift
      ;;
    --compile)
      COMPILE=true
      shift
      ;;
    --dryrun)
      echo "$0: dryrun"
      DRYRUN=--dryrun
      shift
      ;;
    --)
      shift
      break
      ;;
  esac
done

[ -z "$DOWNLOAD" ] && [ -z "$COMPILE" ] && echo "$0: Choose --download or --compile" && exit 1
[ -n "$DOWNLOAD" ] && [ -n "$COMPILE" ] && echo "$0: Options --download and --compile are mutually exclusive" && exit 1

if [ -n "$DOWNLOAD" ];
then

  echo "$0: Removing custom-compiled Mavlink"
  $MY_PATH/mavlink.sh --remove $DRYRUN

  echo "$0: Downloading Mavros"
  [ -z "$DRYRUN" ] && sudo apt -y install ros-melodic-mavros ros-melodic-mavlink ros-melodic-libmavconn ros-melodic-mavros-extras

fi

if [ -n "$COMPILE" ];
then

  echo "$0: Removing pre-installed Mavros and Mavlink"
  [ -z "$DRYRUN" ] && sudo apt -y remove ros-melodic-mavros* ros-melodic-mavlink* ros-melodic-libmavconn ros-melodic-mavros-extras

  echo "$0: Running custom Mavlink install script"
  $MY_PATH/mavlink.sh --install $DRYRUN

fi
