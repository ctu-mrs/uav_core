#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

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
  [ -z "$DRYRUN" ] && ( sudo apt-get -y install ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavlink ros-$ROS_DISTRO-libmavconn ros-$ROS_DISTRO-mavros-extras || echo "Failed to remove pre-installed Mavros" )

fi

if [ -n "$COMPILE" ];
then

  echo "$0: Removing pre-installed Mavros and Mavlink"
  [ -z "$DRYRUN" ] && ( sudo apt-get -y remove ros-$ROS_DISTRO-mavros* ros-$ROS_DISTRO-mavlink* ros-$ROS_DISTRO-libmavconn ros-$ROS_DISTRO-mavros-extras || echo "Failed to remove pre-installed Mavros" )

  echo "$0: Running custom Mavlink install script"
  [ -z "$DRYRUN" ] && $MY_PATH/mavlink.sh --install $DRYRUN

  echo "$0: Installing geographic lib"
  [ -z "$DRYRUN" ] && ( sudo $MY_PATH/../../ros_packages/mavros/mavros/scripts/install_geographiclib_datasets.sh || sudo $MY_PATH/../../ros_packages/mavros/mavros/scripts/install_geographiclib_datasets.sh || sudo $MY_PATH/../../ros_packages/mavros/mavros/scripts/install_geographiclib_datasets.sh || echo "$0: \e[1;31mGeographic lib installation failed even after several attempts. This often happends due to poor network connectivity.\e[0m\n")

fi
