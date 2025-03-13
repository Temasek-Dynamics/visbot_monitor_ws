#!/bin/bash

source /opt/ros/noetic/setup.bash
source ~/ros_ws/devel/setup.bash

if [ -z "$1" ]; then
  echo "Usage: $0 <FOLDER_PATH>"
  exit 1
fi

FOLDER_PATH=$1

# Find the script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIME_STAMP=$(date +"%Y-%m-%d_%H-%M-%S")
ROSBAG_NAME="rosbag_$TIME_STAMP"
ROSBAG_ADDRESS="$FOLDER_PATH/$ROSBAG_NAME"
TOPICS=$(grep -oP '\-\s*\K[^\s]+' "${SCRIPT_DIR}/rostopics.yaml" | tr '\n' ' ')

# rosbag record -O "$ROSBAG_ADDRESS" $(cat /home/visbot/UserData/scripts/rostopics.txt)
roslaunch "${SCRIPT_DIR}/record_rosbag.launch" prefix:=$ROSBAG_ADDRESS topics:="$TOPICS" > "${SCRIPT_DIR}/record_rosbag.log" 2>&1 &

# Echo the latest rosbag
echo "$ROSBAG_NAME is recorded in $FOLDER_PATH"