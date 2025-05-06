#!/bin/bash

launch_file="$1"
algorithm="$2"

# Verify file exists
if [ ! -f "$launch_file" ]; then
    echo "Error: File $launch_file not found"
    exit 1
fi

# Process based on algorithm choice
case "$algorithm" in
    "mebt"|"elev")
        echo "Configuring for $algorithm algorithm (using standard costmap)"
        sed -i '
            # Uncomment standard params if commented
            /<rosparam file="$(find traversability_mapping)\/launch\/params\/move_base\/local_costmap_params.yaml"/ {
                s|<!-- ||g
                s| -->||g
            }
            # Comment trav params if uncommented
            /<rosparam file="$(find traversability_mapping)\/launch\/params\/move_base\/local_costmap_params_trav.yaml"/ {
                /^[[:space:]]*<!--/! s|^|<!-- |
                /^[[:space:]]*-->/! s|$| -->|
            }
        ' "$launch_file"
        ;;
    "trav")
        echo "Configuring for traversability algorithm (using trav costmap)"
        sed -i '
            # Comment standard params if uncommented
            /<rosparam file="$(find traversability_mapping)\/launch\/params\/move_base\/local_costmap_params.yaml"/ {
                /^[[:space:]]*<!--/! s|^|<!-- |
                /^[[:space:]]*-->/! s|$| -->|
            }
            # Uncomment trav params if commented
            /<rosparam file="$(find traversability_mapping)\/launch\/params\/move_base\/local_costmap_params_trav.yaml"/ {
                s|<!-- ||g
                s| -->||g
            }
        ' "$launch_file"
        ;;
    *)
        echo "Error: Unknown algorithm '$algorithm'"
        echo "Supported algorithms: mebt, elev, trav"
        exit 1
        ;;
esac

