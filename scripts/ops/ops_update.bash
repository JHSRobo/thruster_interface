
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
rm -rf motion_control rov_sim pilot_gui gripper_control img_capture

# Clone new packages
git clone https://github.com/JHSRobo/img_capture

# Update dependencies
sudo -u jhsrobo rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y --os=ubuntu:noble

# Give jhsrobo ownership of the workspace
sudo chown jhsrobo: -R /home/jhsrobo/corews

echo "Remember to source ~/.bashrc and compile!"
