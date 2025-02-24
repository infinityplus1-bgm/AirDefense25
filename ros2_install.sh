#!/bin/bash

# update and upgrade the system

sudo apt update && sudo apt upgrade -y


# set system locale 

locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# enable required repos

sudo apt install software-properties-common -y
sudo add-apt-repository universe

# download the ros2 repo gpg key

sudo apt update && sudo apt install curl git -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg


# add the ros2 repo to the system

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null


# update with the new repo

sudo apt update

# install ros2


sudo apt install ros-jazzy-desktop -y

# add the ros2 environment to the bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc


source ~/.bashrc

# install rosdep
sudo apt install python3-colcon-common-extensions -y


sudo apt install python3-rosdep -y


# install python pip

sudo apt install python3-pip -y

# install python virtualenv
sudo apt install python3-virtualenv -y


# install vscode for development

curl "https://code.visualstudio.com/sha/download?build=stable&os=linux-deb-x64" -o vscode.deb && sudo dpkg -i vscode.deb

# install tmux for terminal management
sudo apt install tmux -y