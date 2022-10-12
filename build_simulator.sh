#!/bin/bash
cd ~/

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt install -y --fix-missing  curl # if you haven't already installed curl

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update

sudo apt install -y --fix-missing  ros-melodic-desktop-full

apt search ros-melodic

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install -y --fix-missing python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

sudo apt install -y --fix-missing  python-rosdep

rosdep update

sudo rosdep init

mkdir -p ~/catkin_ws/src

cd ~/catkin_ws/src

sudo apt-get install -y --fix-missing libarmadillo-dev ros-melodic-nlopt

git clone https://github.com/HKUST-Aerial-Robotics/Fast-Planner.git

cd ../ 

source ~/.bashrc

catkin_make

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

source ~/.bashrc
