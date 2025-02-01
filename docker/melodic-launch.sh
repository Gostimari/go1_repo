#!/bin/bash
set -e

#Build the catkin workspace
cd /root/catkin_ws
catkin config --whitelist $BUILDLIST #only builds these packages
catkin build -v
source $SETUP

roslaunch --wait $ROSPACKAGE $LAUNCHFILE #launch the file