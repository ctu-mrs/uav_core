#!/bin/bash
# author: Tomas Baca

# remove the default installation of mavlink, etc.
sudo apt-get -y remove ros-melodic-mavros* ros-melodic-mavlink* ros-melodic-libmavconn

pip install --user future
pip3 install --user future

echo ""
echo #########################
echo STARTING TO BUILD MAVLINK
echo #########################
echo ""

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

# install mavlink headers
cd "$MY_PATH/../../lib/mavlink/"
mkdir build
cd build
cmake ../
make
sudo make install
cd ..
rm -rf build

# copy the ros makefile and config
sudo rm -rf /usr/share/mavlink
sudo mkdir /usr/share/mavlink
sudo cp -r ros/* /usr/share/mavlink/

echo ""
echo #################
echo MAVLINK WAS BUILT
echo #################
echo ""
