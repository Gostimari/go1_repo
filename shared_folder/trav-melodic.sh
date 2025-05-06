#!/bin/bash

WORKDIR=/root/shared_folder
# Define the file and string to monitor
FILE="$WORKDIR/noetic_trav.log"
STRING_MEBT="mebt noetic ready"
STRING_ELEV="elev noetic ready"
STRING_TRAV="trav noetic ready"

echo "Waiting for file $FILE to be created..."

# Wait indefinitely until the file exists
while [ ! -f "$FILE" ]; do
    sleep 1
done

echo "File found. Monitoring for strings ..."

#WORKDIR=/root/catkin_ws/src/

# Check for each string and execute corresponding command
if grep -q "$STRING_MEBT" "$FILE"; then
    echo "String '$STRING_MEBT' detected."
    ./change_local_static_map.sh /root/catkin_ws/src/traversability_mapping/traversability_mapping/launch/params/move_base/local_costmap_params.yaml /fused_costmap &
    ./change_local_costmap_file.sh /root/catkin_ws/src/traversability_mapping/traversability_mapping/launch/include/move_base.launch mebt &

    echo "Target string detected. Starting ROS launch..."
    sleep 5
    roslaunch traversability_mapping offline.launch &
    # Remove the string from file to prevent repeated execution
    #sed -i "/$STRING_MEBT/d" "$FILE"
fi

if grep -q "$STRING_ELEV" "$FILE"; then
    echo "String '$STRING_ELEV' detected."
    ./change_local_static_map.sh /root/catkin_ws/src/traversability_mapping/traversability_mapping/launch/params/move_base/local_costmap_params.yaml /elevation_map_fused_visualization/elevation_grid &
    ./change_local_costmap_file.sh /root/catkin_ws/src/traversability_mapping/traversability_mapping/launch/include/move_base.launch elev &

    echo "Target string detected. Starting ROS launch..."
    sleep 5
    roslaunch traversability_mapping offline.launch &
    # Remove the string from file to prevent repeated execution
    #sed -i "/$STRING_ELEV/d" "$FILE"
fi

if grep -q "$STRING_TRAV" "$FILE"; then
    echo "String '$STRING_TRAV' detected."
    #./change_local_static_map.sh /root/catkin_ws/src/traversability_mapping/traversability_mapping/launch/params/move_base/local_costmap_params.yaml /occupancy_map_local &
    ./change_local_costmap_file.sh /root/catkin_ws/src/traversability_mapping/traversability_mapping/launch/include/move_base.launch trav &

    echo "Target string detected. Starting ROS launch..."
    sleep 5
    roslaunch traversability_mapping offline_trav.launch &
    # Remove the string from file to prevent repeated execution
    #sed -i "/$STRING_TRAV/d" "$FILE"
fi

sleep 5

WORKDIR=/root/shared_folder
echo "melodic ready" >> $WORKDIR/melodic_trav.log