#!/bin/bash 
#--Robotics Lab --Boston University
#This script will install all PX4 targets for simulation, ROS, Gazebo
#to install ROS, visit http://wiki.ros.org/Documentation

#Add the user in dialout group 
sudo usermod -a -G dialout $USER
#remove the serial modem manager which interfaces heavily with robotics related use of serial port/USP port. 
sudo apt-get remove modemmanager
#Update the package list and install the following dependencies for all PX4 build targets.
sudo apt-get update -y
sudo apt-get install git zip qtcreator cmake build-essential genromfs ninja-build exiftool vim-common -y

#required python package
sudo apt-get install python-argparse python-empy python-toml python-numpy python-yaml python-dev python-pip -y
sudo -H pip install --upgrade pip 
sudo -H pip install pandas jinja2 pyserial cerberus
sudo -H pip install pyulog

#build system 
sudo apt-get install ninja-build -y
## Gazebo dependencies
sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y
#Install gazebo_ros_pkgs
sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control

#eProsima Fast RTPS is a C++ implementation of the RTPS (Real Time Publish Subscribe) protocol
wget http://www.eprosima.com/index.php/component/ars/repository/eprosima-fast-rtps/eprosima-fast-rtps-1-5-0/eprosima_fastrtps-1-5-0-linux-tar-gz -O eprosima_fastrtps-1-5-0-linux.tar.gz
tar -xzf eprosima_fastrtps-1-5-0-linux.tar.gz eProsima_FastRTPS-1.5.0-Linux/
tar -xzf eprosima_fastrtps-1-5-0-linux.tar.gz requiredcomponents
tar -xzf requiredcomponents/eProsima_FastCDR-1.0.7-Linux.tar.gz
#j<number_of_cpu_cores_in_your_system> to speed up the compilation of the libraries
(cd eProsima_FastCDR-1.0.7-Linux && ./configure --libdir=/usr/lib && make -j2 && sudo make install)
(cd eProsima_FastRTPS-1.5.0-Linux && ./configure --libdir=/usr/lib && make -j2 && sudo make install)
rm -rf requiredcomponents eprosima_fastrtps-1-5-0-linux.tar.gz

#mavros 
sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras

#install GeographicLib
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
<<<<<<< HEAD
=======


sleep 5 
sudo apt-get update 
sudo apt-get upgrade

