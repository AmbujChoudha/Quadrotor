#!/bin/bash

source $HOME/.bashrc

firm=$HOME/src/Firmware

cd $firm
source $firm/Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo
roslaunch px4 multi_uav_mavros_sitl.launch




