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
            roslaunch traversability_mapping offline.launch &
            COMMAND_PID=$!
            UNSAFE_COMMAND_USED=1  # Mark that an unsafe command was used
            ;;
        2)
            echo "Custom command clicked"
            cmd=$(yad --entry --title="Custom Command" --width=400 --text="Type a command:")
            if [[ -n "$cmd" ]]; then
                # Generate a unique log file name based on the current timestamp
                LOG_FILE="custom_output_melodic_$(date +%s).log"

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
            UNSAFE_COMMAND_USED=2  # Mark that an unsafe command was used
            ;;
        3)
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
                1)
                    send_safe_stop
                    UNSAFE_COMMAND_USED=3  # Mark that an unsafe command was used
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
                1)
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
        --button="Custom Command:2" \
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

