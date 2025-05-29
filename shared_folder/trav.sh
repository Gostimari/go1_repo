#!/bin/bash

WORKDIR=/root/shared_folder
# Define the file and string to monitor
FILE="$WORKDIR/melodic_trav.log"
STRING="melodic ready"

# Make sure Wi-Fi is turned off
sudo nmcli radio wifi off

# Properly connect to the correct ethernet ports
sudo nmcli connection up RS-Bpearl

sleep 2

# Start ROS core if not already running
roscore &

sleep 2  # Give some time for roscore to start

# Launch the first ROS launch file
roslaunch go1_ros_interface robot.launch connection_type:=ethernet feedback_frequency:=50 &

sleep 2

roslaunch realsense2_camera rs_aligned_depth.launch &

sleep 2

# Launch the third ROS launch file
roslaunch rslidar_sdk start.launch &

sleep 10

roslaunch ig_lio lio_velodyne_Bpearl.launch &
sleep 10

echo "trav noetic ready" >> $WORKDIR/noetic_trav.log

echo "Waiting for file $FILE to be created..."

# Wait indefinitely until the file exists
while [ ! -f "$FILE" ]; do
    sleep 1
done

echo "File found. Monitoring for string '$STRING'..."

# Wait indefinitely until the string is found in the file
while ! grep -q "$STRING" "$FILE"; do
    sleep 1
done

rosrun metrics_extractor metrics.py &
sleep 5

#echo "Target string detected. Starting ROS launch..."
roslaunch gps_waypoint_nav gps_waypoint_nav.launch &
