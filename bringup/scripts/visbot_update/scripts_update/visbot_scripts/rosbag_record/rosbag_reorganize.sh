#!/bin/bash

base_dir="$HOME/UserData/bags"

# Find all ROS bag files in the base directory
find "$base_dir" -maxdepth 1 -type f \( -name "rosbag_*.bag" -o -name "rosbag_*.bag.active" -o -name "rosbag_*.bag.orig.active" \) | while read -r file; do
    # Extract the date from the file name
    filename=$(basename "$file")
    file_date=$(echo "$filename" | grep -oP '\d{4}-\d{2}-\d{2}')
    date_dir="$base_dir/$file_date"

    if [ ! -d "$date_dir" ]; then
        mkdir -p "$date_dir"
    fi

    # Move the file to the date directory
    mv "$file" "$date_dir/"
done

echo "Files reorganized by date in $base_dir."