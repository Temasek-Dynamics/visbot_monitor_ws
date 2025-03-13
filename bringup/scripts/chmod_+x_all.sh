#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/noetic/setup.bash
source $SCRIPT_DIR/../../devel/setup.bash

monitor_package="visbot_monitor"
monitor_path=$(rospack find $monitor_package)/scripts

gui_package="swarm_gui"
gui_path=$(rospack find $gui_package)

# Find and give execute permission to all .sh files in the current directory and subdirectories
# find "$SCRIPT_DIR" -type f -name "*.sh" -exec echo {} \;
find "$SCRIPT_DIR" -type f -name "*.sh" -exec chmod +x {} \;
echo "All .sh files in $SCRIPT_DIR are executable."

# Check if the scripts directory exists in the visbot_monitor package, then give execute permission to all .py files
if [ -d "$monitor_path" ]; then
    # find "$monitor_path" -type f -name "*.py" -exec echo {} \;
    find "$monitor_path" -type f -name "*.py" -exec chmod +x {} \;
    echo "All .py in $monitor_path are executable."
else
    echo "No 'scripts' directory found in $monitor_package."
fi

if [ -d "$gui_path" ]; then
    find "$gui_path" -type f -name "*.py" -exec chmod +x {} \;
    echo "All .py in $gui_path are executable."
else
    echo "No 'scripts' directory found in $gui_package."
fi

echo "Permission Execution Completed."