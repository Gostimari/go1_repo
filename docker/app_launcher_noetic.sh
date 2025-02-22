#!/bin/bash

# Global variables
COMMAND_PID=""
UNSAFE_COMMAND_USED=""  # Flag to track if an unsafe command has been run

# Function to send zero velocity and null goal for safe stopping
send_safe_stop() {
  echo "Sending zero velocity command to /cmd_vel"
  rostopic pub -1 --use-rostime /cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
  
  echo "Sending null goal to move_base to cancel any active goals"
  rostopic pub -1 --use-rostime /move_base/cancel actionlib_msgs/GoalID -- {}
}

# Function to display command output in real-time
show_command_output() {
  local log_file="$1"
  # Display the command output with tail and two buttons
  tail -f "$log_file" | yad --title="Command Output" --width=600 --height=400 \
      --text-info --listen --button="Stop":9 --button="Back":10
  local ret=$?
  
  if [ $ret -eq 9 ]; then
    # "Stop" button clicked
    if [ -n "$COMMAND_PID" ]; then
      echo "Stopping command with PID $COMMAND_PID"
      kill -SIGINT -- -$COMMAND_PID
      COMMAND_PID=""
    fi
    # Immediately show the static log file content in a new YAD window
    yad --title="Command Output (Stopped)" --width=600 --height=400 \
        --text-info --filename="$log_file" --button="Back":6
  elif [ $ret -eq 10 ]; then
    # "Back" button clicked
    if [ -n "$COMMAND_PID" ]; then
      echo "Stopping command with PID $COMMAND_PID"
      kill -SIGINT -- -$COMMAND_PID
      COMMAND_PID=""
    fi
  fi
}

execute_command() {
    case "$1" in
        2)
            echo "Collect GPS clicked"
            roslaunch ig_lio noetic_main_collect.launch &
            COMMAND_PID=$!
            UNSAFE_COMMAND_USED=2  # Mark that an unsafe command was used
            ;;
        3)
            echo "MEBT clicked"
            roslaunch ig_lio noetic_main_mebt.launch &
            COMMAND_PID=$!
            UNSAFE_COMMAND_USED=3  # Mark that an unsafe command was used
            ;;
        4)
            echo "Elevation clicked"
            roslaunch ig_lio noetic_main_elev.launch &
            COMMAND_PID=$!
            UNSAFE_COMMAND_USED=4  # Mark that an unsafe command was used
            ;;
        5)
            echo "Traversability clicked"
            roslaunch ig_lio noetic_main_trav.launch &
            COMMAND_PID=$!
            UNSAFE_COMMAND_USED=5  # Mark that an unsafe command was used
            ;;
        6)  
            echo "Startup drivers Script clicker"
            ./startup.sh &
            COMMAND_PID=$!
            UNSAFE_COMMAND_USED=6  # Mark that an unsafe command was used
            ;;
        7)
            echo "Custom command clicked"
            cmd=$(yad --entry --title="Custom Command" --width=400 --text="Type a command:")
            if [[ -n "$cmd" ]]; then
                # Generate a unique log file name based on the current timestamp
                LOG_FILE="custom_output_noetic_$(date +%s).log"

                # Clear the new log file (though it shouldn't exist yet)
                > "$LOG_FILE"

                # Write the command and separator to the log file
                echo "Command Executed: $cmd" >> "$LOG_FILE"
                echo "---------------------" >> "$LOG_FILE"

                # Run the command and append its output to the log file
                eval "stdbuf -oL -eL $cmd >> $LOG_FILE 2>&1 &"
                COMMAND_PID=$!

                # Display the log file in YAD
                show_command_output "$LOG_FILE"
            fi
            UNSAFE_COMMAND_USED=7  # Mark that an unsafe command was used
            ;;
        8)
            # Kill existing process if any
            if [ -n "$COMMAND_PID" ]; then
                echo "Sending SIGINT to process group $COMMAND_PID"
                kill -SIGINT -- -$COMMAND_PID
                COMMAND_PID=""
                # Kill all ROS nodes and processes
                echo "Killing all ROS nodes and roscore..."
                rosnode kill -a 2>/dev/null
                sleep 1
                killall -9 roscore rosmaster rosout 2>/dev/null
                pkill -9 -f "ros/master" 2>/dev/null
                pkill -9 -f "ros/launch" 2>/dev/null
            else
                echo "No specific process to kill."
            fi
            case "$UNSAFE_COMMAND_USED" in
                3|4|5)
                    send_safe_stop
                    UNSAFE_COMMAND_USED=8  # Mark that an unsafe command was used
                    ;;
                *)
                    echo "UNSAFE_COMMAND_USED=$UNSAFE_COMMAND_USED; skipping safe stop"
                    ;;
            esac
            ;;
        252)
            echo "Window closed by user. Exiting."
            # Kill existing process if any
            if [ -n "$COMMAND_PID" ]; then
                echo "Sending SIGINT to process group $COMMAND_PID"
                kill -SIGINT -- -$COMMAND_PID
                COMMAND_PID=""
                # Kill all ROS nodes and processes
                echo "Killing all ROS nodes and roscore..."
                rosnode kill -a 2>/dev/null
                sleep 1
                killall -9 roscore rosmaster rosout 2>/dev/null
                pkill -9 -f "ros/master" 2>/dev/null
                pkill -9 -f "ros/launch" 2>/dev/null
            else
                echo "No specific process to kill."
            fi
            case "$UNSAFE_COMMAND_USED" in
                3|4|5)
                    send_safe_stop
                    UNSAFE_COMMAND_USED=252  # Mark that an unsafe command was used
                    ;;
                *)
                    echo "UNSAFE_COMMAND_USED=$UNSAFE_COMMAND_USED; skipping safe stop"
                    ;;
            esac
            echo "Cleaning up all log files..."
            rm -f custom_output_*.log  # Remove all log files matching the pattern
            exit 0
            ;;
        *)
            echo "Unknown button clicked: $1"
            ;;
    esac
}

while true; do
    # Run YAD and capture its exit code immediately.
    yad --title "Roslaunch Noetic App" \
        --form \
        --width=300 --height=200 \
        --button="Collect GPS:2" \
        --button="MEBT:3" \
        --button="Elevation:4" \
        --button="Traversability:5" \
        --button="Startup Drivers:6" \
        --button="Custom Command:7" \
        --button="Kill:8" \
        --buttons-layout=spread \
        --text="Click a button to execute a command or Kill to stop the process. If you want to launch the traversability_mapping, you need to launch it on the Roslaunch Melodic App too. The 'Custom Command' button is to launch a terminal command, like 'rostopic list' with built-in terminal output."
    
    BUTTON_EXIT_CODE=$?
    
    execute_command "$BUTTON_EXIT_CODE"
    
    # If the exit code is 252 (window closed), then exit.
    if [ "$BUTTON_EXIT_CODE" -eq 252 ]; then
        echo "Window closed by user. Exiting."
        exit 0
    fi
done
