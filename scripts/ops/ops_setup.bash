
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
echo "alias opside=\"ros2 launch core opside.yaml\"" >> /home/jhsrobo/.bashrc
echo "alias depinstall=\"rosdep install --from-paths /home/jhsrobo/corews/src --ignore-src\"" >> /home/jhsrobo/.bashrc
echo "alias sym=\"colcon build --symlink-install\""  >> /home/jhsrobo/.bashrc
echo "export PYTHONWARNINGS=ignore" >> /home/jhsrobo/.bashrc
echo "export PIP_BREAK_SYSTEM_PACKAGES=1" >> /home/jhsrobo/.bashrc


# Add Network Shortcuts
echo "192.168.1.100 master" >> /etc/hosts
echo "192.168.1.110 opside" >> /etc/hosts
echo "192.168.1.111 bottomside" >> /etc/hosts

# Install ROS
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
apt update
apt install ros-dev-tools
apt upgrade
apt install ros-jazzy-desktop

# Install GitHub cli tools
type -p curl >/dev/null || (sudo apt update && sudo apt install curl -y)
curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg | sudo dd of=/usr/share/keyrings/githubcli-archive-keyring.gpg
sudo chmod go+r /usr/share/keyrings/githubcli-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null
apt update
apt install gh -y

# Call the update script
sudo bash /home/jhsrobo/corews/scripts/ops/ops_update.bash
