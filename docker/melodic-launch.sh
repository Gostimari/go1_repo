#!/bin/bash
set -e

#Build the catkin workspace
cd /root/catkin_ws
catkin config --whitelist $BUILDLIST #only builds these packages
catkin build -v

source $SETUP

exec /bin/bash

#!/bin/bash
# while true; do
#     echo "Service is running..."
#     sleep 1
# done
#roslaunch --wait $ROSPACKAGE $LAUNCHFILE #launch the file