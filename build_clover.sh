#!/usr/bin/env bash

set -e

echo "--- Current environment:"
/usr/bin/env

echo "Enabling passwordless sudo"
echo "${PASSWORD}" | sudo -E -S sh -c "echo '${USER} ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers"

echo "--- Increasing apt retries"
sudo -E sh -c 'echo "APT::Acquire::Retries \"3\";" > /etc/apt/apt.conf.d/80-retries'
cat /etc/apt/apt.conf.d/80-retries

echo "--- Allowing apt to perform its updates"
sudo -E sh -c 'apt update; while fuser /var/lib/dpkg/lock ; do sleep 0.5 ; done'

echo "--- Installing ROS desktop packages"
sudo -E sh -c 'apt update; apt install -y python-rosdep python-rosinstall-generator python-wstool build-essential ros-melodic-desktop'

echo "--- Downloading PX4 and installing its dependencies"
git clone -b v1.10.1-clover https://github.com/CopterExpress/Firmware ${HOME}/Firmware
sudo -E -S sh -c '${HOME}/Firmware/Tools/setup/ubuntu.sh'
sudo -E -S sh -c 'echo "2" | update-alternatives --config java'
sudo -E -S sed -i -e '/^assistive_technologies=/s/^/#/' /etc/java-*-openjdk/accessibility.properties

echo "--- Prebuilding PX4 SITL configuration"
make -C ${HOME}/Firmware px4_sitl
echo "--- Patching gazebo plugins for SITL"
cd ${HOME}/Firmware/Tools/sitl_gazebo
patch -p1 < /tmp/patches/sitl_gazebo.patch
echo 'export SVGA_VGPU10=0' >> ${HOME}/Firmware/Tools/setup_gazebo.bash

echo "--- Cloning and installing Clover packages"
mkdir -p ${HOME}/catkin_ws/src
git clone https://github.com/CopterExpress/clover ${HOME}/catkin_ws/src/clover
git clone https://github.com/CopterExpress/ros_led ${HOME}/catkin_ws/src/ros_led
# Make PX4 and Gazebo plugins visible in the workspace
ln -s ${HOME}/Firmware ${HOME}/catkin_ws/src/Firmware
ln -s ${HOME}/Firmware/Tools/sitl_gazebo ${HOME}/catkin_ws/src/sitl_gazebo
rosdep install --from-paths ${HOME}/catkin_ws/src --ignore-src --rosdistro melodic -y
curl https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh -o ${HOME}/install_geographiclib_datasets.sh
chmod a+x ${HOME}/install_geographiclib_datasets.sh
sudo -E sh -c '${HOME}/install_geographiclib_datasets.sh'
sudo /usr/bin/python2.7 -m pip install -r ${HOME}/catkin_ws/src/clover/clover/requirements.txt
source /opt/ros/melodic/setup.bash
cd ${HOME}/catkin_ws && catkin_make
echo "source ${HOME}/catkin_ws/devel/setup.bash" >> ~/.bashrc

echo "--- Installing npm"
cd ${HOME}
wget --progress=dot:giga https://nodejs.org/dist/v10.15.0/node-v10.15.0-linux-x64.tar.gz
tar -xzf node-v10.15.0-linux-x64.tar.gz
sudo cp -R node-v10.15.0-linux-x64/* /usr/local/
rm -rf node-v10.15.0-linux-x64
rm node-v10.15.0-linux-x64.tar.gz
echo "--- Reconfiguring npm to use local prefix"
mkdir ${HOME}/.npm-global
npm config set prefix "${HOME}/.npm-global"
export PATH=${HOME}/.npm-global/bin:$PATH
echo 'export PATH='${HOME}'/.npm-global/bin:$PATH' >> ${HOME}/.bashrc
echo "--- Installing gitbook and building docs"
cd ${HOME}/catkin_ws/src/clover
NPM_CONFIG_UNSAFE_PERM=true npm install gitbook-cli -g
NPM_CONFIG_UNSAFE_PERM=true gitbook install
gitbook build
touch node_modules/CATKIN_IGNORE docs/CATKIN_IGNORE _book/CATKIN_IGNORE clover/www/CATKIN_IGNORE # ignore documentation files by catkin

echo "--- Enabling roscore service"
sed -i "s/pi/${USER}/g" ${HOME}/catkin_ws/src/clover/builder/assets/roscore.service
sudo cp ${HOME}/catkin_ws/src/clover/builder/assets/roscore.service /etc/systemd/system
sudo systemctl enable roscore.service

echo "--- Installing QGroundControl"
sudo -E sh -c "usermod -a -G dialout $USER"
sudo -E sh -c 'apt remove -y modemmanager; apt install -y gstreamer1.0-plugins-bad gstreamer1.0-libav'
curl https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage -o ${HOME}/QGroundControl.AppImage
chmod a+x ${HOME}/QGroundControl.AppImage

echo "--- Installing additional packages"
sudo -E sh -c 'apt update; apt install -y sshfs gvfs-fuse gvfs-backends python3-opencv byobu ipython ipython3 byobu nmap lsof tmux vim ros-melodic-rqt-multiplot'
