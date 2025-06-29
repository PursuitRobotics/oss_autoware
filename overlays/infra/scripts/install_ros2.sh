# /!/usr/bin/env bash

# Install ROS 2 Humble on Ubuntu 20.04 (Focal Fossa)
# This script is intended to be run on a fresh Ubuntu 22.04 installation.

wget -O /tmp/amd64.env https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64.env && source /tmp/amd64.env

# Taken from: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

# You will need to add the ROS 2 apt repository to your system. First, make sure that the Ubuntu Universe repository is enabled by checking the output of this command.
apt-cache policy | grep universe

# The output of the above command should contain following:

# 500 http://us.archive.ubuntu.com/ubuntu focal/universe amd64 Packages
#     release v=20.04,o=Ubuntu,a=focal,n=focal,l=Ubuntu,c=universe,b=amd64

# If you don’t see an output line like the one above, then enable the Universe repository with these instructions.
sudo apt install software-properties-common
sudo add-apt-repository universe

# Now add the ROS 2 apt repository to your system. First authorize our GPG key with apt.
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Then add the repository to your sources list.
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update your apt repository caches after setting up the repositories.
sudo apt update

# Desktop Install
ros2_installation_type=desktop
sudo apt install ros-${rosdistro}-${ros2_installation_type}

# Environment setup
# (Optional) You can source ros2 in the ~/.bashrc file.
echo '' >> ~/.bashrc && echo "source /opt/ros/${rosdistro}/setup.bash" >> ~/.bashrc