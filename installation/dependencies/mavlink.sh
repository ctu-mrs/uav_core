#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

options=$(getopt -l "install,remove,dryrun" -o "" -a -- "$@")

eval set -- "$options"

while true
do
  case $1 in
    --install)
      INSTALL=true
      shift
      ;;
    --remove)
      REMOVE=true
      shift
      ;;
    --dryrun)
      echo "$0: dryrun"
      DRYRUN=true
      shift
      ;;
    --)
      shift
      break
      ;;
  esac
done

[ -z "$INSTALL" ] && [ -z "$REMOVE" ] && echo "$0: Choose --install or --remove" && exit 1
[ -n "$INSTALL" ] && [ -n "$REMOVE" ] && echo "$0: Options --install and --remove are mutually exclusive" && exit 1

if [ -n "$INSTALL" ];
then

  echo "$0: Installing future"
  [ -z "$DRYRUN" ] && sudo pip install --user future
  [ -z "$DRYRUN" ] && sudo -H pip install --user future
  [ -z "$DRYRUN" ] && sudo pip3 install --user future
  [ -z "$DRYRUN" ] && sudo -H pip3 install --user future
  [ -z "$DRYRUN" ] && sudo apt -y install python-future python3-future

  echo "$0: Checking out the desired release"
  [ -z "$DRYRUN" ] && cd "$MY_PATH/../../lib/mavlink-gbp-release/"
  [ -z "$DRYRUN" ] && # git checkout a131e4bd665d2dc7f822797faf13d783fcd4bb8a # release/melodic/mavlink/2019.5.20-1
  [ -z "$DRYRUN" ] && bloom-generate rosdebian --os-name ubuntu --ros-distro melodic

  echo "$0: Building mavlink"
  [ -z "$DRYRUN" ] && [ ! -e build ] && mkdir build
  [ -z "$DRYRUN" ] && cd build
  [ -z "$DRYRUN" ] && cmake ../
  [ -z "$DRYRUN" ] && make

  echo "$0: Installing mavlink"
  [ -z "$DRYRUN" ] && sudo make install

  echo "$0: Cleaning after Mavlink compilation"
  [ -z "$DRYRUN" ] && cd "$MY_PATH/../../lib/mavlink-gbp-release/"
  [ -z "$DRYRUN" ] && git clean -fd

fi

if [ -n "$REMOVE" ];
then

  LOCATION=/usr/local/share/mavlink
  [ -z "$DRYRUN" ] && [ -e "$LOCATION" ] && (echo "$0: Removing $LOCATION" && sudo rm -r "$LOCATION") || echo "$0: Nothing to remove"

fi
