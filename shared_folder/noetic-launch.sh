#!/bin/bash
set -e
FILE=/etc/apt/sources.list.d/ros1-latest.list
# rm /etc/apt/sources.list.d/ros1-latest.list
# if [ -f "$FILE" ]; then
#        rm /etc/apt/sources.list.d/ros1-latest.list
#        echo "Removed ros source file"
# fi
# sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

apt update

apt install ros-noetic-serial ros-noetic-rviz-imu-plugin -y

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
