#!/bin/bash

export GDK_BACKEND=x11
export XDG_SESSION_TYPE=x11

# Global variables
COMMAND_PID=""
UNSAFE_COMMAND_USED="" # Flag to track if an unsafe command has been run

WORKDIR=/root/shared_folder

# Function to display command output in real-time
show_command_output() {
    local log_file="$1"
    # Display the command output with tail and two buttons
    tail -f "$log_file" | yad --title="Command Output" --width=600 --height=400 \
        --text-info --listen --button="Stop":4 --button="Back":5
    local ret=$?

    if [ $ret -eq 4 ]; then
        # "Stop" button clicked
        if [ -n "$COMMAND_PID" ]; then
            echo "Stopping command with PID $COMMAND_PID"
            kill -SIGINT -- -$COMMAND_PID
            COMMAND_PID=""
        fi
        # Immediately show the static log file content in a new YAD window
        yad --title="Command Output (Stopped)" --width=600 --height=400 \
            --text-info --filename="$log_file" --button="Back":6
    elif [ $ret -eq 5 ]; then
        # "Back" button clicked
        if [ -n "$COMMAND_PID" ]; then
            echo "Stopping command with PID $COMMAND_PID"
            kill -SIGINT -- -$COMMAND_PID
            COMMAND_PID=""
        fi
    fi
}

# Function to execute commands based on button clicked
execute_command() {
    case $1 in
    1)
        echo "Traversability clicked"
        ./trav-melodic.sh &
        COMMAND_PID=$!
        UNSAFE_COMMAND_USED=1 # Mark that an unsafe command was used
        ;;
    6)
        echo "Catkin Build clicked"
        cd ../catkin_ws
        SHELL=/bin/bash command catkin build &
        cd ../shared_folder
        COMMAND_PID=$!
        UNSAFE_COMMAND_USED=6 # Mark that an unsafe command was used
        ;;
    7)
        echo "Ros Purge clicked"
        rosclean purge -y
        ;;
    8)
        echo "xsens driver clicked"
        roslaunch xsens_driver xsens_driver.launch &
        COMMAND_PID=$!
        UNSAFE_COMMAND_USED=8 # Mark that an unsafe command was used
        ;;
    2)
        echo "Custom command clicked"
        cmd=$(yad --entry --title="Custom Command" --width=400 --text="Type a command:")
        if [[ -n "$cmd" ]]; then
            # Generate a unique log file name based on the current timestamp
            LOG_FILE="custom_output_melodic_$(date +%s).log"

            # Clear the new log file (though it shouldn't exist yet)
            >"$LOG_FILE"

            # Write the command and separator to the log file
            echo "Command Executed: $cmd" >>"$LOG_FILE"
            echo "---------------------" >>"$LOG_FILE"

            # Run the command and append its output to the log file
            eval "stdbuf -oL -eL $cmd >> $LOG_FILE 2>&1 &"
            COMMAND_PID=$!

            # Display the log file in YAD
            show_command_output "$LOG_FILE"
        fi
        UNSAFE_COMMAND_USED=2 # Mark that an unsafe command was used
        ;;
    3)
        # Kill existing process if any
        if [ -n "$COMMAND_PID" ]; then
            case "$UNSAFE_COMMAND_USED" in
                1)
                    send_safe_stop
                    UNSAFE_COMMAND_USED=3  # Mark that an unsafe command was used
                    ;;
                *)
                    echo "UNSAFE_COMMAND_USED=$UNSAFE_COMMAND_USED; skipping safe stop"
                    ;;
            esac
            echo "Sending SIGINT to process group $COMMAND_PID"
            kill -SIGINT -- -$COMMAND_PID
            COMMAND_PID=""
        else
            echo "No specific process to kill."
        fi
        # Kill all ROS nodes and processes
        echo "Killing all ROS nodes and roscore..."
        rosnode kill --all 2>/dev/null &
        # Force-kill roscore/rosmaster
        killall -9 roscore rosmaster roslaunch &
        # Kill Gazebo servers and clients
        pkill -9 -f "rosout" &
        pkill -9 -f "gzserver\|gzclient" &
        killall -9 gzserver gzclient &
        pkill -9 -f "junior_ctrl" &
        pkill -9 -f "gps_waypoint_nav" &
        pkill -9 -f "ig_lio" &
        pkill -9 -f "robot_state_publisher" &
        pkill -9 -f "go1_ros_interface" &
        pkill -9 -f "realsense2_camera" &
        pkill -9 -f "rslidar_sdk" &
        # Clear zombie processes
        ps -aux | grep -E 'defunct|Z' | awk '{print $2}' | xargs kill -9 2>/dev/null
        rosclean purge -y
        rm -f $WORKDIR/noetic_trav.log
        rm -f $WORKDIR/melodic_trav.log
        rm -f $WORKDIR/gps_waypoint.log
        rm -f $WORKDIR/fail.log
        ;;
    252)
        # Kill existing process if any
        if [ -n "$COMMAND_PID" ]; then
            case "$UNSAFE_COMMAND_USED" in
                1)
                    send_safe_stop
                    UNSAFE_COMMAND_USED=3  # Mark that an unsafe command was used
                    ;;
                *)
                    echo "UNSAFE_COMMAND_USED=$UNSAFE_COMMAND_USED; skipping safe stop"
                    ;;
            esac
            echo "Sending SIGINT to process group $COMMAND_PID"
            kill -SIGINT -- -$COMMAND_PID
            COMMAND_PID=""
        else
            echo "No specific process to kill."
        fi
        # Kill all ROS nodes and processes
        echo "Killing all ROS nodes and roscore..."
        rosnode kill --all 2>/dev/null &
        # Force-kill roscore/rosmaster
        killall -9 roscore rosmaster roslaunch &
        # Kill Gazebo servers and clients
        pkill -9 -f "rosout" &
        pkill -9 -f "gzserver\|gzclient" &
        killall -9 gzserver gzclient &
        pkill -9 -f "junior_ctrl" &
        pkill -9 -f "gps_waypoint_nav" &
        pkill -9 -f "ig_lio" &
        pkill -9 -f "robot_state_publisher" &
        pkill -9 -f "go1_ros_interface" &
        pkill -9 -f "realsense2_camera" &
        pkill -9 -f "rslidar_sdk" &
        # Clear zombie processes
        ps -aux | grep -E 'defunct|Z' | awk '{print $2}' | xargs kill -9 2>/dev/null
        rosclean purge -y
        rm -f $WORKDIR/noetic_trav.log
        rm -f $WORKDIR/melodic_trav.log
        rm -f $WORKDIR/gps_waypoint.log
        rm -f $WORKDIR/fail.log
        exit 0
        ;;
    *)
        echo "Unknown button clicked"
        ;;
    esac
}

# Loop to re-open the dialog after each button click
while true; do
    yad --title "Roslaunch Melodic App" \
        --form \
        --width=300 --height=200 \
        --button="Traversability:1" \
        --button="xsens driver:8"  \
        --button="Catkin Build:6" \
        --button="Custom Command:2" \
        --button="Ros Purge:7" \
        --button="Kill:3" \
        --buttons-layout=spread \
        --text="Click a button to execute a command or Kill to stop the process. The 'Custom Command' button is to launch a terminal command, like 'rostopic list' with built-in terminal output."

    BUTTON_EXIT_CODE=$?

    execute_command $BUTTON_EXIT_CODE

    if [ "$BUTTON_EXIT_CODE" -eq 252 ]; then
        echo "Window closed by user. Exiting."
        exit 0
    fi
done
