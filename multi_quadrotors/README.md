The purpose of the repisotory is simulate single quadrotor in Gazebo 

## to begin 
First install the toolchain from https://dev.px4.io/en/setup/getting_started.html



git clone the PX4 firmware:

[PX4 Firmware](https://github.com/PX4/Firmware)

It also requires to have MAVRos globally on the operating system

[Mavros Installation](https://dev.px4.io/en/ros/mavros_installation.html)

## the use of bash scripts
### Launching Gazebo
To launch gazebo, navigate to the folder, and run the bash script, launch_gazebo.sh
```
$ ./launch_sitlgazebo.sh
```
This scripts will run the ros master with the gazebo model of the quadrotor and MAVRos
### Controlling the Quad

to make sure MAVRos is communicating with the topics by running in a new terminal. This will show all the topics. 
```
$ rostopic list
```

