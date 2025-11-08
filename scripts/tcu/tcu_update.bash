#!/bin/bash

# Check sudo perms
if [[ "$(id -u)" != 0 ]]
  then echo "Please run as root"
  exit
fi

# Update the corews
git pull --hard

# Remove previously compiled code
rm -rf /home/jhsrobo/corews/build
rm -rf /home/jhsrobo/corews/install
rm -rf /home/jhsrobo/corews/log

cd /home/jhsrobo/corews/src

# Remove old packages
rm -rf motion_control rov_sim pilot_gui gripper_control

# Clone new packages
git clone https://github.com/JHSRobo/motion_control
git clone https://github.com/JHSRobo/rov_sim
git clone https://github.com/JHSRobo/pilot_gui
git clone https://github.com/JHSRobo/gripper_control

# Update dependencies
cd /home/jhsrobo/corews
sudo -u jhsrobo rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y --os=ubuntu:noble

# Give jhsrobo ownership of the workspace
sudo chown jhsrobo: -R /home/jhsrobo/corews

# Install phidget packages
curl -fsSL https://www.phidgets.com/downloads/setup_linux | bash
apt install -y libphidget22
pip install Phidget22

echo "Remember to source ~/.bashrc and compile!"
