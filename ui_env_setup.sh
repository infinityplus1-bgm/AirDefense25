#!/bin/bash


# for setting up the enviroment for running the ui


# install pyqt5 through apt

sudo apt install python3-pyqt5 -y


# install opencv

sudo apt install python3-opencv -y


# install cv_bridge


sudo apt install ros-jazzy-cv-bridge -y


# install QSwitchControl for some ui widget needed

pip install QSwitchControl --break-system-packages # we have to do this because of the way ros and python work :(
