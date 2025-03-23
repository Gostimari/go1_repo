#!/bin/bash

cleanup() {
    echo "Stopping all processes..."
    # Only kill processes if they were started
    # [[ -n "$DOCKER_PID" ]] && kill -SIGINT $DOCKER_PID
    [[ -n "$ROSBAG_PID" ]] && kill -SIGINT $ROSBAG_PID
    [[ -n "$LAUNCH1_PID" ]] && kill -SIGINT $LAUNCH1_PID
    [[ -n "$LAUNCH2_PID" ]] && kill -SIGINT $LAUNCH2_PID
    [[ -n "$LAUNCH3_PID" ]] && kill -SIGINT $LAUNCH3_PID
    [[ -n "$LAUNCH4_PID" ]] && kill -SIGINT $LAUNCH4_PID
    [[ -n "$ROSCORE_PID" ]] && kill -SIGINT $ROSCORE_PID
    wait
    echo "All processes stopped."
    exit 0  # Ensure the script exits after cleanup
}

# Setup trap to handle Ctrl+C and other exit scenarios
trap cleanup SIGINT SIGTERM

# Make sure Wi-Fi is turned off
sudo nmcli radio wifi off

# Properly connect to the correct ethernet ports
sudo nmcli connection up RS-Bpearl

sleep 2

# Start ROS core if not already running
roscore &
ROSCORE_PID=$!
sleep 2  # Give some time for roscore to start

# Launch the first ROS launch file
roslaunch go1_ros_interface robot.launch connection_type:=ethernet feedback_frequency:=50 &
LAUNCH1_PID=$!

# Launch the second ROS launch file
roslaunch realsense2_camera rs_aligned_depth.launch &
LAUNCH2_PID=$!

# Launch the third ROS launch file
roslaunch rslidar_sdk start.launch &
LAUNCH3_PID=$!

sleep 15

# Inform user about how to stop the script
echo "Press Ctrl+C to stop everything"

# Keep script running until user triggers cleanup
while :
do
    sleep 0.01
done
