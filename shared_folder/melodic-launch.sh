#!/bin/bash
set -e

#export ROS_MASTER_URI=http://192.168.12.173:11311/
#export ROS_IP=192.168.12.173

#Build the catkin workspace
cd /root/catkin_ws
catkin config --whitelist $BUILDLIST #only builds these packages
catkin build -v

#Run emlid configuration device script
python /root/catkin_ws/src/depends/ethzasl_xsens_driver/nodes/mtdevice.py -l --output-mode=sotac --output-settings=tqMAG

#Start melodic app
cd ../shared_folder
exec ./app_launcher_melodic.sh

# To prevent the container to auto stop
# while true; do
#     echo "Service is running..."
#     sleep 1
# done
