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
BAG_DIR="${HOME}/UserData/bags/${test_date}"

# Check if the directory exists
if [ ! -d "$BAG_DIR" ]; then
    echo "Directory $BAG_DIR does not exist."
    exit 1
else
    echo "rosbag located at: $BAG_DIR"
fi

# Find all .active files in the directory, excluding .orig.active files
active_files=$(find "$BAG_DIR" -type f -name "*.active" ! -name "*.orig.active")

# Check if any .active files are found
if [ -z "$active_files" ]; then
    echo "No .active files found in $BAG_DIR."
    exit 0
else
    echo "rosbags should be fixed:" $active_files
fi

# Process each .active file
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

echo "All files processed."