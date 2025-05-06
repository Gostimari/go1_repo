#!/bin/bash

directory="$1"
new_topic="$2"

# Validate the new topic name (ROS rules: a-z, A-Z, 0-9, /, _)
if [[ ! "$new_topic" =~ ^[/a-zA-Z0-9_]+$ ]]; then
    echo "ERROR: Invalid ROS topic name '$new_topic'"
    echo "Valid characters: a-z, A-Z, 0-9, /, _"
    exit 1
fi

# Ensure the topic starts with a forward slash (ROS convention)
if [[ ! "$new_topic" =~ ^/ ]]; then
    new_topic="/$new_topic"
fi

# Use alternative sed delimiter (|) to avoid issues with slashes
find "$directory" -type f -name "*.yaml" -exec sed -i "s|^\([[:space:]]*map_topic:\)[[:space:]]*[^[:space:]]*|\\1 $new_topic|" {} +