#!/bin/bash

rostopic echo -n 1 /move_base/status > last_status_raw.yaml
GOAL_ID=$(grep "id:" last_status_raw.yaml | head -n 1 | awk '{print $2}' | tr -d '"')
GOAL_STAMP_SECS=$(grep "stamp:" -A 1 last_status_raw.yaml | grep secs | awk '{print $2}')
GOAL_STAMP_NSECS=$(grep "stamp:" -A 2 last_status_raw.yaml | grep nsecs | awk '{print $2}')
rostopic pub /move_base/status actionlib_msgs/GoalStatusArray "status_list:
- goal_id:
    stamp: {secs: $GOAL_STAMP_SECS, nsecs: $GOAL_STAMP_NSECS}
    id: '$GOAL_ID'
  status: 3
  text: 'Goal reached successfully'" -1 &

rm last_status_raw.yaml

# rostopic echo -n 1 /move_base/status | sed 's/status: [0-9]/status: 3/' | rostopic pub /move_base/status actionlib_msgs/GoalStatusArray -



# rostopic echo -n 1 /move_base/status | tee status_msg.yaml
# sed 's/status: [0-9]/status: 3/' status_msg.yaml > modified_status.yaml
# rostopic pub /move_base/status actionlib_msgs/GoalStatusArray < modified_status.yaml

# rostopic echo -n 1 /move_base/status | \
# sed 's/status: [0-9]/status: 3/' | \
# rostopic pub /move_base/status actionlib_msgs/GoalStatusArray - -1

#!/bin/bash

# Step 1: Capture the last status message
# rostopic echo -n 1 /move_base/status > last_status_raw.yaml

# Step 2: Extract header fields
# HEADER_STAMP_SECS=$(grep -m 1 "stamp:" -A 1 last_status_raw.yaml | grep secs | awk '{print $2}')
# HEADER_STAMP_NSECS=$(grep -m 1 "stamp:" -A 2 last_status_raw.yaml | grep nsecs | awk '{print $2}')
# FRAME_ID=$(grep -m 1 "frame_id" last_status_raw.yaml | awk -F": " '{print $2}' | tr -d '"')

# # Step 3: Extract goal_id, stamp, id, and text from the raw message
# GOAL_STAMP_SECS=$(grep "stamp:" -A 1 last_status_raw.yaml | grep secs | awk '{print $2}')
# GOAL_STAMP_NSECS=$(grep "stamp:" -A 2 last_status_raw.yaml | grep nsecs | awk '{print $2}')
# GOAL_ID=$(grep "id:" last_status_raw.yaml | head -n 1 | awk '{print $2}' | tr -d '"')
# TEXT=$(grep "text:" last_status_raw.yaml | head -n 1 | cut -d':' -f2- | sed 's/^ *//')

# Step 4: Build the full message
# MSG="header:
#   stamp: {secs: $HEADER_STAMP_SECS, nsecs: $HEADER_STAMP_NSECS}
#   frame_id: '$FRAME_ID'
# status_list:
# - goal_id:
#     stamp: {secs: $GOAL_STAMP_SECS, nsecs: $GOAL_STAMP_NSECS}
#     id: '$GOAL_ID'
#   status: 3
#   text: 'Modified from: $TEXT'"

# # Step 5: Publish the modified message
# rostopic pub /move_base/status actionlib_msgs/GoalStatusArray "$MSG" -1

