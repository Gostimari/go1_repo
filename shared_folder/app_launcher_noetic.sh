#!/bin/bash

export GDK_BACKEND=x11
export XDG_SESSION_TYPE=x11

# Global variables
COMMAND_PID=""
UNSAFE_COMMAND_USED=""  # Flag to track if an unsafe command has been run

WORKDIR=/root/shared_folder

# Function to send zero velocity and null goal for safe stopping
send_safe_stop() {
  echo "Sending null goal to move_base to cancel any active goals"
  rostopic pub -1 --use-rostime /move_base/cancel actionlib_msgs/GoalID -- {}

  echo "Sending zero velocity command to /cmd_vel"
  rostopic pub -1 --use-rostime /cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
}

# Function to display command output in real-time
show_command_output() {
  local log_file="$1"
  # Display the command output with tail and two buttons
  tail -f "$log_file" | yad --title="Command Output" --width=600 --height=400 \
      --text-info --listen --button="Stop":12 --button="Back":13
  local ret=$?
  
  if [ $ret -eq 12 ]; then
    # "Stop" button clicked
    if [ -n "$COMMAND_PID" ]; then
      echo "Stopping command with PID $COMMAND_PID"
      kill -SIGINT -- -$COMMAND_PID
      COMMAND_PID=""
    fi
    # Immediately show the static log file content in a new YAD window
    yad --title="Command Output (Stopped)" --width=600 --height=400 \
        --text-info --filename="$log_file" --button="Back":6
  elif [ $ret -eq 13 ]; then
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
            ./collect_gps.sh &
            COMMAND_PID=$!
            UNSAFE_COMMAND_USED=2  # Mark that an unsafe command was used
            ;;
        3)
            echo "MEBT clicked"
            ./mebt.sh &
            COMMAND_PID=$!
            UNSAFE_COMMAND_USED=3 # Mark that an unsafe command was used
            ;;
        4)
            echo "Elevation clicked"
            ./elevation.sh &
            COMMAND_PID=$!
            UNSAFE_COMMAND_USED=4  # Mark that an unsafe command was used
            ;;
        5)
            echo "Traversability clicked"
            ./trav.sh &
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
            echo "Catkin Build clicked"
            cd ../catkin_ws
            SHELL=/bin/bash command catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release &
            cd ../shared_folder
            COMMAND_PID=$!
            UNSAFE_COMMAND_USED=13 # Mark that an unsafe command was used
            ;;
        8)
            echo "Rqt Tf Tree clicked"
            rosrun rqt_tf_tree rqt_tf_tree &
            COMMAND_PID=$!
            UNSAFE_COMMAND_USED=14 # Mark that an unsafe command was used
            ;;
        9)
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
        10)
            echo "Ros Purge clicked"
            rosclean purge -y
            ;;
        12)
            echo "Rosbag Record clicked"
            ./record_bag.sh
            ;;
        13)
            echo "Goal Status clicked"
            ./goal_status.sh
            ;;
        11)
            # Kill existing process if any
            if [ -n "$COMMAND_PID" ]; then
                case "$UNSAFE_COMMAND_USED" in
                    3|4|5)
                        send_safe_stop
                        UNSAFE_COMMAND_USED=8  # Mark that an unsafe command was used
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
            killall -9 roscore rosmaster roslaunch rosout &
            # Kill Gazebo servers and clients
            pkill -9 -f "gzserver\|gzclient" &
            killall -9 gzserver gzclient &
            pkill -9 -f "junior_ctrl" &
            pkill -9 -f "gps_waypoint_nav" &
            pkill -9 -f "ig_lio" &
            pkill -9 -f "robot_state_publisher" &
            pkill -9 -f "navigation_final_semfire_pilot" &
            pkill -9 -f "nodelet" &
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
            rm -f $WORKDIR/custom_output*
            ;;
        252)
            echo "Window closed by user. Exiting."
            # Kill existing process if any
            if [ -n "$COMMAND_PID" ]; then
                case "$UNSAFE_COMMAND_USED" in
                    3|4|5)
                        send_safe_stop
                        UNSAFE_COMMAND_USED=252  # Mark that an unsafe command was used
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
            killall -9 roscore rosmaster roslaunch rosout &
            # Kill Gazebo servers and clients
            pkill -9 -f "gzserver\|gzclient" &
            killall -9 gzserver gzclient &
            pkill -9 -f "junior_ctrl" &
            pkill -9 -f "gps_waypoint_nav" &
            pkill -9 -f "ig_lio" &
            pkill -9 -f "robot_state_publisher" &
            pkill -9 -f "navigation_final_semfire_pilot" &
            pkill -9 -f "nodelet" &
            pkill -9 -f "go1_ros_interface" &
            pkill -9 -f "realsense2_camera" &
            pkill -9 -f "rslidar_sdk" &
            # Clear zombie processes
            ps -aux | grep -E 'defunct|Z' | awk '{print $2}' | xargs kill -9 2>/dev/null
            echo "Cleaning up all log files..."
            rm -f custom_output_*.log # Remove all log files matching the pattern
            rosclean purge -y
            rm -f $WORKDIR/noetic_trav.log
            rm -f $WORKDIR/melodic_trav.log
            rm -f $WORKDIR/gps_waypoint.log
            rm -f $WORKDIR/fail.log
            rm -f $WORKDIR/custom_output*
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
        --button="Catkin Build:7" \
        --button="Rqt tf tree:8" \
        --button="Custom Command:9" \
        --button="Ros Purge:10" \
        --button="Rosbag:12" \
        --button="Goal_Status:13" \
        --button="Kill:11" \
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
