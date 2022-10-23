#!/bin/bash
cd ~/

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install -y --fix-missing  curl # if you haven't already installed curl

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update

sudo apt install -y --fix-missing  ros-noetic-desktop-full

apt search ros-noetic

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install -y --fix-missing python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

sudo apt install -y --fix-missing  python-rosdep

rosdep update

sudo rosdep init

mkdir -p ~/catkin_ws/src

cd ~/catkin_ws/src

sudo apt-get install -y --fix-missing libarmadillo-dev ros-noetic-nlopt

git clone https://github.com/HKUST-Aerial-Robotics/Fast-Planner.git

cd ../ 

source ~/.bashrc

cd ~/catkin_ws
ls
cd src/
sudo apt install -y catkin
sudo apt-get install -y ros-noetic-catkin python-catkin-tools 

sudo apt remove python-catkin-pkg
sudo apt install -y python-catkin-pkg

sudo apt-get install -y ros-noetic-cv-camera
sudo apt-get install -y ros-noetic-web-video-server
sudo apt-get install -y ros-noetic-mavros ros-noetic-mavros-extras
sudo apt install -y ros-noetic-pcl-conversions
sudo apt install -y ros-noetic-cmake-modules
sudo apt install -y ros-noetic-pcl-ros
sudo apt install -y ros-noetic-tf2-geometry-msgs
sudo apt install -y ros-noetic-laser-geometry
sudo apt install -y ros-noetic-rviz
sudo apt install -y ros-noetic-led-msgs

sudo apt-get install -y python-rospy
sudo apt-get install -y python3-rospy
sudo apt install -y libopencv-dev
sudo apt-get install -y ros-noetic-usb-cam

sudo apt install -y ros-noetic-realsense2-camera ros-noetic-realsense2-camera-dbgsym ros-noetic-realsense2-description 

sudo apt-get install -y ros-noetic-tf2-ros
cd ~/catkin_ws/src
git clone https://github.com/RobotWebTools/tf2_web_republisher

cd ~/catkin_ws/src
git clone https://github.com/CopterExpress/clover.git clover

cd ~/catkin_ws/src/clover/clover/udev
sudo cp 99-px4fmu.rules /lib/udev/rules.d

cd ~
curl https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh | sudo bash
chmod a+x ./install_geographiclib_datasets.sh

#cd ~/catkin_ws/src
#git clone https://github.com/CopterExpress/ros_led
#git clone https://github.com/okalachev/vl53l1x_ros
#rosdep install --from-paths src --ignore-src -y
#rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y --os=ubuntu:xenial
#rosdep install --from-paths src --ignore-src -y

source ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

cd ~/catkin_ws
catkin_make
#sudo apt-get install qv4l2
