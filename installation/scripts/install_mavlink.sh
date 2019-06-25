#!/bin/bash
# author: Tomas Baca

# remove the default installation of mavlink, etc.
sudo apt-get -y remove ros-melodic-mavros* ros-melodic-mavlink* ros-melodic-libmavconn

sudo pip install --user future
sudo -H pip install --user future
sudo pip3 install --user future
sudo -H pip3 install --user future
sudo apt -y install python-future python3-future

echo ""
echo #########################
echo STARTING TO BUILD MAVLINK
echo #########################
echo ""

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

# install mavlink headers

# pull the correct version of mavlink
cd "$MY_PATH/../../lib/mavlink-gbp-release/"
# git checkout a131e4bd665d2dc7f822797faf13d783fcd4bb8a # release/melodic/mavlink/2019.5.20-1
bloom-generate rosdebian --os-name ubuntu --ros-distro melodic

# build it
mkdir build
cd build
cmake ../
make

# install it
sudo make install

cd "$MY_PATH/../../lib/mavlink-gbp-release/"
git clean -fd

echo ""
echo #################
echo MAVLINK WAS BUILT
echo #################
echo ""
