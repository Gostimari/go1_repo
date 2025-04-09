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

# Launch the third ROS launch file
roslaunch rslidar_sdk start.launch &

sleep 10
roslaunch xsens_driver xsens_driver.launch &

sleep 10

roslaunch ig_lio lio_velodyne_Bpearl.launch &
sleep 10
roslaunch octomap_server octomap_mapping.launch &
sleep 2
roslaunch navigation_final_semfire_pilot ranger_navigation.launch &
sleep 35
rosrun metrics_extractor metrics.py &
sleep 5
roslaunch gps_waypoint_nav gps_waypoint_nav.launch &
