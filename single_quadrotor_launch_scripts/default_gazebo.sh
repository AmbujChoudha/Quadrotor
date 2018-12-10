#!/bin/bash

source $HOME/.bashrc

firm=$HOME/src/Firmware

cd $firm

no_sim=1 make posix_sitl_default gazebo






