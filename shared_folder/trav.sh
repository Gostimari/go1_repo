#!/bin/bash

WORKDIR=/root/shared_folder
# Define the file and string to monitor
FILE="$WORKDIR/melodic_trav-log.txt"
STRING="melodic ready"

roslaunch unitree_gazebo robot_simulation.launch &
sleep 15
rosrun unitree_guide junior_ctrl &
sleep 10

echo "noetic ready" >>$WORKDIR/noetic_trav-log.txt

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

sleep 35
rosrun metrics_extractor metrics.py &
sleep 5

#echo "Target string detected. Starting ROS launch..."
roslaunch gps_waypoint_nav gps_waypoint_nav.launch &
