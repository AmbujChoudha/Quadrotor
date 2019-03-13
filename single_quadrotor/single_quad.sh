#!/bin/bash

source $HOME/.bashrc

firm=$HOME/Firmware
cd $firm
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo
roslaunch px4 posix_sitl.launch

