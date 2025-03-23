#!/bin/bash

roslaunch ig_lio lio_velodyne_Bpearl.launch &
sleep 10
roslaunch elevation_mapping_demos_sim go1_elevation_sim.launch &
sleep 35
rosrun metrics_extractor metrics.py &
sleep 5
roslaunch gps_waypoint_nav gps_waypoint_nav.launch &
