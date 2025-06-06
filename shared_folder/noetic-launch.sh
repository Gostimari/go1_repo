#!/bin/bash
set -e

apt install ros-noetic-serial -y

cd /root/catkin_ws/
catkin config --buildlist $BUILDLIST
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release

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

apt install zsh -y

cd ../shared_folder
exec ./app_launcher_noetic.sh
#exec /bin/bash
#!/bin/bash
# while true; do
#     echo "Service is running..."
#     sleep 1
# done
#roslaunch $ROSPACKAGE $LAUNCHFILE
