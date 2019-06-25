sudo apt-get update -y
sudo apt-get upgrade -y

UAV_CORE_PATH=`dirname "$0"`
UAV_CORE_PATH=`( cd "$UAV_CORE_PATH" && pwd )`

cd "$UAV_CORE_PATH"

./installation/scripts/install_dependencies.sh

git clean -fd
git reset --hard
git submodule deinit -f .
git submodule sync
git submodule update --init --recursive
gitman install --force

# compile mavlink
cd "$UAV_CORE_PATH/installation/scripts"
./install_mavlink.sh

# source installation/scripts/download_binaries.sh
# echo "binaries downloaded and configured"
cd $UAV_CORE_PATH
ROS_WORKSPACE=~/mrs_workspace
rm -rf $ROS_WORKSPACE

mkdir -p $ROS_WORKSPACE/src
ln -s $UAV_CORE_PATH $ROS_WORKSPACE/src/uav_core
cd $ROS_WORKSPACE
source /opt/ros/melodic/setup.bash

#sudo rosdep init
#sudo rosdep fix-permissions
rosdep update

catkin init

catkin config --profile debug --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_CXX_FLAGS='-std=c++17 -march=native -fno-diagnostics-color' -DCMAKE_C_FLAGS='-march=native -fno-diagnostics-color'
catkin profile set debug

catkin build -j1
