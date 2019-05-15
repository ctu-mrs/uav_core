#!/bin/bash
# author: Tomas Baca

# remove the default installation of mavlink, etc.
sudo apt-get -y remove ros-melodic-mavros* ros-melodic-mavlink* ros-melodic-libmavconn

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

sudo apt-get -y install python3-pip
pip install --user future

cd "$MY_PATH/../../lib/mavlink/"
mkdir build
cd build
cmake ../
make
sudo make install
cd ..
rm -rf build
