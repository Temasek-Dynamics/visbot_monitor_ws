#!/bin/bash

source /opt/ros/noetic/setup.bash
source ~/ros_ws/devel/setup.bash

DATE=$(date +"%Y-%m-%d")

FOLDER_PATH="$HOME/UserData/bags/$DATE"
mkdir -p "$FOLDER_PATH"

# Find the script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIME_STAMP=$(date +"%Y-%m-%d_%H-%M-%S")
ROSBAG_NAME="rosbag_$TIME_STAMP"
ROSBAG_ADDRESS="$FOLDER_PATH/$ROSBAG_NAME"
TOPICS=$(grep -oP '\-\s*\K[^\s]+' "${SCRIPT_DIR}/rostopics.yaml" | tr '\n' ' ')

# Echo the latest log
LOGDIR="$HOME/log"
LATESTLOG=$(ls -d "$LOGDIR"/log* 2>/dev/null | sort -V | tail -n 1)

if [ -z "$LATESTLOG" ]; then
    echo "#######################################################"
    echo "No log folders found."
else
    LOGNUM=$(basename "$LATESTLOG" | grep -oP '\d+')
    echo "#######################################################"
    echo "The latest log number is: $LOGNUM"
fi

# Echo the latest rosbag
echo "The latest rosbag number is: $ROSBAG_NAME"
echo "#######################################################"
# rosbag record -O "$ROSBAG_ADDRESS" $(cat /home/visbot/UserData/scripts/rostopics.txt)
roslaunch "${SCRIPT_DIR}/rosbag_record.launch" prefix:=$ROSBAG_ADDRESS topics:="$TOPICS" > "${SCRIPT_DIR}/rosbag_record.log" 2>&1 &