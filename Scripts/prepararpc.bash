#!/bin/bash

build_ws(){
    cd ~/ros_ws
    catkin_make
}

sudo apt-get install ros-indigo-desktop-full
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
catkin_init_workspace
build_ws
cd ~/ros_ws/src
git clone https://github.com/ros-drivers/rosserial.git
git clone https://github.com/ros-perception/pointcloud_to_laserscan.git
build_ws
echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc
sudo apt-get install ros-indigo-openni-camera
sudo apt-get install ros-indigo-openni-launch
