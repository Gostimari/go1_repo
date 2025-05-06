#!/bin/bash

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

# Launch the second ROS launch file
#roslaunch realsense2_camera rs_aligned_depth.launch &

# Launch the third ROS launch file
roslaunch rslidar_sdk start.launch &