#!/bin/bash
# author: Robert Penicka
set -e

echo "Starting install preparation"
openssl aes-256-cbc -K $encrypted_d9693e727195_key -iv $encrypted_d9693e727195_iv -in ./.ci/deploy_key_github.enc -out ./.ci/deploy_key_github -d
eval "$(ssh-agent -s)"
chmod 600 ./.ci/deploy_key_github
ssh-add ./.ci/deploy_key_github
sudo apt-get update -qq
sudo apt-get install dpkg git python-setuptools python3-setuptools python3-pip

echo "running the main install.sh"
./installation/install.sh

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
ln -s $TRAVIS_BUILD_DIR
source /opt/ros/melodic/setup.bash
cd ~/catkin_ws

echo "install part ended"
