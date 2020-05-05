#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "\"${last_command}\" command failed with exit code $?"' ERR

options=$(getopt -l "install,remove:,dryrun" -o "" -a -- "$@")

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
      DRYRUN=true
      shift
      ;;
  esac
  shift
done

if [ -n "$INSTALL" ];
then

  [ -z "$DRYRUN" ] && sudo pip install --user future
  [ -z "$DRYRUN" ] && sudo -H pip install --user future
  [ -z "$DRYRUN" ] && sudo pip3 install --user future
  [ -z "$DRYRUN" ] && sudo -H pip3 install --user future
  [ -z "$DRYRUN" ] && sudo apt -y install python-future python3-future

  # get the path to this script
  MY_PATH=`dirname "$0"`
  MY_PATH=`( cd "$MY_PATH" && pwd )`

  # install mavlink headers

  # pull the correct version of mavlink
  [ -z "$DRYRUN" ] && cd "$MY_PATH/../../lib/mavlink-gbp-release/"
  [ -z "$DRYRUN" ] && # git checkout a131e4bd665d2dc7f822797faf13d783fcd4bb8a # release/melodic/mavlink/2019.5.20-1
  [ -z "$DRYRUN" ] && bloom-generate rosdebian --os-name ubuntu --ros-distro melodic

  echo "Building mavlink"
  [ -z "$DRYRUN" ] && [ ! -e build ] && mkdir build
  [ -z "$DRYRUN" ] && cd build
  [ -z "$DRYRUN" ] && cmake ../
  [ -z "$DRYRUN" ] && make

  echo "Installing mavlink"
  [ -z "$DRYRUN" ] && sudo make install

  echo "Cleaning after Mavlink compilation"
  [ -z "$DRYRUN" ] && cd "$MY_PATH/../../lib/mavlink-gbp-release/"
  [ -z "$DRYRUN" ] && git clean -fd

fi

if [ -n "$INSTALL" ];
then

  LOCATION=/usr/local/share/mavlink
  [ -z "$DRYRUN" ] && [ -e $LOCATION ] && echo "Removing $LOCATION" && sudo rm -r "$LOCATION"

fi

exit 0
