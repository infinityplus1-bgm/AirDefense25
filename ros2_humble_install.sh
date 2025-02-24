#!/bin/bash

# Update and upgrade the system
sudo apt update && sudo apt upgrade -y

# Set system locale
locale  # Check for UTF-8
sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Enable required repos
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

# Download the ROS 2 repo GPG key
sudo apt update && sudo apt install curl git -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the ROS 2 repo to the system
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update with the new repo
sudo apt update

# Install ROS 2 Humble
sudo apt install ros-humble-desktop -y

# Add the ROS 2 environment to the bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Source the updated bashrc
source ~/.bashrc

# Install rosdep
sudo apt install python3-colcon-common-extensions -y
sudo apt install python3-rosdep -y

# Install python pip
sudo apt install python3-pip -y

# Install python virtualenv
sudo apt install python3-virtualenv -y

# install vscode for development

curl "https://code.visualstudio.com/sha/download?build=stable&os=linux-deb-x64" -o vscode.deb && sudo dpkg -i vscode.deb

# Install tmux for terminal management
sudo apt install tmux -y
