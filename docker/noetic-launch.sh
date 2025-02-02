#!/bin/bash
set -e

cd /root/catkin_ws/
catkin config --buildlist $BUILDLIST
catkin build

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

#!/bin/bash
# while true; do
#     echo "Service is running..."
#     sleep 1
# done
#roslaunch --wait $ROSPACKAGE $LAUNCHFILE