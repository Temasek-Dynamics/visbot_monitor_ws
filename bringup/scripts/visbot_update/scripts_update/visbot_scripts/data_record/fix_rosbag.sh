#!/bin/bash

source /opt/ros/noetic/setup.bash
source ~/ros_ws/devel/setup.bash

# Check if test_date is provided
if [ -z "$1" ]; then
    echo "Usage: $0 <test_date>"
    exit 1
fi
test_date=$1

# Define the directory containing the bag files
BASE_DIR="${HOME}/UserData/experiments/${test_date}"

# Check if the directory exists
if [ ! -d "$BASE_DIR" ]; then
    echo "Directory $BASE_DIR does not exist."
    exit 1
else
    echo "Base directory located at: $BASE_DIR"
fi

for exp_dir in "$BASE_DIR"/exp*; do
    if [ -d "$exp_dir" ]; then
        echo "Processing directory: $exp_dir"

        # Find all .active files in the directory, excluding .orig.active files
        active_files=$(find "$exp_dir" -type f -name "*.active" ! -name "*.orig.active")

        if [ -z "$active_files" ]; then
            echo "No .active files found in $exp_dir."
            continue
        else
            echo "rosbags should be fixed: $active_files"
        fi

        for active_file in $active_files; do
            echo "Processing $active_file..."

            # Reindex the .active file
            rosbag reindex "$active_file"
            if [ $? -ne 0 ]; then
                echo "Failed to reindex $active_file"
                continue
            fi

            fixed_file="${active_file%.active}"
            rosbag fix "$active_file" "$fixed_file"

            if [ $? -ne 0 ]; then
                echo "Failed to fix $active_file"
                continue
            fi

            rm -f "$active_file"
            orig_active_file="${active_file%.active}.orig.active"
            if [ -f "$orig_active_file" ]; then
                rm -f "$orig_active_file"
            fi

            echo "Successfully fixed $active_file to $fixed_file"
        done

        echo "All files in $exp_dir processed."
    fi
done

echo "All files processed."