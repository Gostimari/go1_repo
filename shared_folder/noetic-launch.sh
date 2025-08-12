#!/bin/bash
set -e

#export ROS_MASTER_URI=http://192.168.12.173:11311
#export ROS_IP=192.168.12.173


#Build the catkin workspace
cd /root/catkin_ws/
catkin config --buildlist $BUILDLIST
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release


#Start noetic app
cd ../shared_folder
exec ./app_launcher_noetic.sh

# To prevent the container to auto stop
# while true; do
#     echo "Service is running..."
#     sleep 1
# done
