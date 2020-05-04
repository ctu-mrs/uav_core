#!/bin/bash
# author: Robert Penicka

echo "Starting install preparation" 
openssl aes-256-cbc -K $encrypted_d9693e727195_key -iv $encrypted_d9693e727195_iv -in ./.ci/deploy_key_github.enc -out ./.ci/deploy_key_github -d
eval "$(ssh-agent -s)"
chmod 600 ./.ci/deploy_key_github
ssh-add ./.ci/deploy_key_github
sudo apt-get update -qq
sudo apt-get install dpkg git expect python-setuptools python3-setuptools python3-pip
./installation/scripts/install_ros.sh
./installation/scripts/install_dependencies.sh
echo "dependencies installed"
git clean -fd
git reset --hard
git submodule deinit -f .
git submodule sync
git submodule update --init --recursive
gitman install --force
echo "gitman submodules downloaded"
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
ln -s $TRAVIS_BUILD_DIR
source /opt/ros/melodic/setup.bash
cd ~/catkin_ws
catkin init
catkin config --profile debug --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_CXX_FLAGS='-std=c++17 -march=native -fno-diagnostics-color' -DCMAKE_C_FLAGS='-march=native -fno-diagnostics-color'
catkin profile set debug
echo "install part ended"

