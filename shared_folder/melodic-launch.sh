#!/bin/bash
set -e

#Build the catkin workspace
cd /root/catkin_ws
catkin config --whitelist $BUILDLIST #only builds these packages
catkin build -v

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

python /root/catkin_ws/src/depends/ethzasl_xsens_driver/nodes/mtdevice.py -l --output-mode=sotac --output-settings=tqMAG

cd ../shared_folder
exec ./app_launcher_melodic.sh

# To prevent the container to auto stop
# while true; do
#     echo "Service is running..."
#     sleep 1
# done
