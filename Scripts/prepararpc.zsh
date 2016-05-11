#!/usr/bin/zsh

#DEPENDENCIAS
sudo apt-get install libaria

#CREANDO WORKSPACE
echo "source /opt/ros/indigo/setup.zsh" >> ~/.zshrc
source ~/.zshrc
cd ~
mkdir -p ros_ws/src
cd ros_ws/src
catkin_init_workspace
cd ~/ros_ws
catkin_make
cd ~/ros_ws/src
git clone https://github.com/amor-ros-pkg/rosaria.git
echo "source ~/ros_ws/devel/setup.zsh" >> ~/.zshrc
source ~/.zshrc
cd ~/ros_ws
catkin_make

#INSTALACIONES
sudo apt-get install python-rosinstall
sudo apt-get install ros-indigo-openni-camera ros-indigo-openni-launch 
sudo apt-get install ros-indigo-turtlebot
