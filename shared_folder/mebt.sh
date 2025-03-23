#!/bin/bash

roslaunch unitree_gazebo robot_simulation.launch &
sleep 20
rosrun unitree_guide junior_ctrl &
sleep 10
roslaunch octomap_server octomap_mapping.launch &
sleep 2
roslaunch navigation_final_semfire_pilot ranger_navigation.launch &
sleep 35
rosrun metrics_extractor metrics.py &
sleep 5
roslaunch gps_waypoint_nav gps_waypoint_nav.launch &
