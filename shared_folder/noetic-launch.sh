#!/bin/bash
set -e

export ROS_MASTER_URI=http://192.168.12.173:11311
export ROS_IP=192.168.12.173


#Build the catkin workspace
cd /root/catkin_ws/
# catkin config --buildlist $BUILDLIST
# catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release

# setup ros environment
if [[ ! -z "${SETUP}" ]]; then
       #from environment variable; should be a absolute path to the appropriate workspaces's setup.bash
       echo "source env is set to '$SETUP'"
else
       # basic ros environment
	export SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
       echo "source env is set to '/opt/ros/$ROS_DISTRO/setup.bash'"
fi
source $SETUP

cd ../shared_folder
exec ./app_launcher_noetic.sh

# To prevent the container to auto stop
# while true; do
#     echo "Service is running..."
#     sleep 1
# done
