#!/bin/bash



echo "Bash Version: ${BASH_VERSION}"

clone=$HOME/Quadrotor


roscore & #>/dev/null &

# Sleep to allow the roscore to finish initializing, otherwise the launch files will not be able to locate the port
sleep 4


x-terminal-emulator -e $clone/launch_ros_gazebo.sh &>/dev/null &
sleep 4
x-terminal-emulator -e $clone/launch_mavros.sh &>/dev/null &

cd $clone/scripts/




















































































































