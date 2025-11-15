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
rm -rf thruster_interface gpio_interface sensor_interface

# Clone new packages
git clone https://github.com/JHSRobo/thruster_interface
git clone https://github.com/JHSRobo/gpio_interface
git clone https://github.com/JHSRobo/sensor_interface
git clone https://github.com/JHSRobo/camera_stream --depth=1 # For some reason, this repo takes forever to clone so I added the --depth=1 flag to speed it up. Fix later.

# Update dependencies
sudo -u jhsrobo rosdep update
sudo -u jhsrobo rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y --os=ubuntu:jazzy

# Install Packages that are NOT RECOGNIZED BY ROSDEP
# You can find lists of all rosdep recognized packages here:
# https://github.com/ros/rosdistro/tree/master/rosdep
# If your package isn't in base.yaml or python.yaml, add it below.
pip install adafruit-circuitpython-bme280 adafruit-circuitpython-sht31d adafruit-circuitpython-ahtx0 adafruit-circuitpython-ina3221 ninja meson smbus2 adafruit_bno055 gpiozero

# Give jhsrobo ownership of the workspace
sudo chown jhsrobo: -R /home/jhsrobo/corews

echo -e "Remember to source ~/.bashrc and compile!\nADDITIONAL NOTE: After running the setup script, reboot to correctly set udev rules for GPIO usage."
