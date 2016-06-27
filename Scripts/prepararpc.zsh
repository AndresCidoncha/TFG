#!/usr/bin/zsh

sudo apt-get install ros-indigo-desktop-full -y
source /opt/ros/indigo/setup.zsh
echo "source /opt/ros/indigo/setup.zsh" >> ~/.zshrc
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
catkin_init_workspace
cd ~/ros_ws
catkin_make
cd ~/ros_ws/src
git clone https://github.com/ros-drivers/rosserial.git
git clone https://github.com/ros-perception/pointcloud_to_laserscan.git
cd ~/ros_ws
catkin_make
source ~/ros_ws/devel/setup.zsh
echo "source ~/ros_ws/devel/setup.zsh" >> ~/.zshrc
sudo apt-get install ros-indigo-openni-camera -y
sudo apt-get install ros-indigo-openni-launch -y
