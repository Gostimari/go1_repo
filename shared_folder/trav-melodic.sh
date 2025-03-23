#!/bin/bash

WORKDIR=/root/shared_folder
# Define the file and string to monitor
FILE="$WORKDIR/noetic_trav-log.txt"
STRING="noetic ready"

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

echo "Target string detected. Starting ROS launch..."

sleep 5

roslaunch traversability_mapping offline.launch &

sleep 5

echo "melodic ready" >> $WORKDIR/melodic_trav-log.txt
