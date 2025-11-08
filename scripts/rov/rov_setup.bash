#!/bin/bash

# Check sudo perms
if [[ "$(id -u)" != 0 ]]
  then echo "Please run as root"
  exit
fi

# Install helpful packages
apt install gcc make perl curl pip wget gnupg -y

# Get rid of Ubuntu error for pip
rm -f /usr/lib/python3.12/EXTERNALLY-MANAGED

# Add helpful stuff to ~/.bashrc
echo "source /opt/ros/jazzy/setup.bash" >> /home/jhsrobo/.bashrc
echo "source /home/jhsrobo/corews/install/setup.bash" >> /home/jhsrobo/.bashrc
echo "alias bottomside=\"ros2 launch core bottomside.yaml\"" >> /home/jhsrobo/.bashrc
echo "alias depinstall=\"rosdep install --from-paths /home/jhsrobo/corews/src --ignore-src\"" >> /home/jhsrobo/.bashrc
echo "alias sym=\"colcon build --symlink-install\""  >> /home/jhsrobo/.bashrc
echo "export PYTHONWARNINGS=ignore" >> /home/jhsrobo/.bashrc
echo "export PIP_BREAK_SYSTEM_PACKAGES=1" >> /home/jhsrobo/.bashrc

# Add Network Shortcuts
echo "192.168.1.100 topside" >> /etc/hosts
echo "192.168.1.110 opside" >> /etc/hosts
echo "192.168.1.111 bottomside" >> /etc/hosts

# Disable annoying service that slows boot
echo "[Service]" >> /etc/systemd/system/systemd-networkd-wait-online.service.d/override.conf
echo "ExecStart=" >> /etc/systemd/system/systemd-networkd-wait-online.service.d/override.conf
echo "ExecStart=/usr/lib/systemd/systemd-networkd-wait-online --eth0" >> /etc/systemd/system/systemd-networkd-wait-online.service.d/override.conf

# Install ROS
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
apt update
apt install ros-dev-tools
apt upgrade
apt install ros-jazzy-desktop

# Install ustreamer 
cd /home/jhsrobo
apt install build-essential libevent-dev libjpeg-dev libbsd-dev
cd /home/jhsrobo
git clone --depth=1 https://github.com/pikvm/ustreamer
cd ustreamer
make
cp ustreamer /usr/local/bin
rm -rf ~/ustreamer

# Give GPIO Permissions
groupadd gpio
usermod -aG gpio jhsrobo
echo 'KERNEL=="gpio*", GROUP="gpio", MODE="0660"' >> /etc/udev/rules.d/99-gpio.rules
udevadm control --reload-rules
udevadm trigger

# Call the update script
bash /home/jhsrobo/corews/scripts/rov/rov_update.bash
