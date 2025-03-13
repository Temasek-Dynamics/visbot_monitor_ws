#!/bin/bash

source /opt/ros/noetic/setup.bash
source ~/ros_ws/devel/setup.bash

FOLDER_PATH="$HOME/UserData/bags/vio_bags"
mkdir -p "$FOLDER_PATH"

# Find the script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIME_STAMP=$(date +"%Y-%m-%d_%H-%M-%S")
ROSBAG_NAME="vio_raw_bag_onboard_$TIME_STAMP"
ROSBAG_ADDRESS="$FOLDER_PATH/$ROSBAG_NAME"
TOPICS=$(grep -oP '\-\s*\K[^\s]+' "${SCRIPT_DIR}/viotopics.yaml" | tr '\n' ' ')

# rosbag record -O "$ROSBAG_ADDRESS" $(cat /home/visbot/UserData/scripts/rostopics.txt)
roslaunch "${SCRIPT_DIR}/vio_raw_data_record.launch" prefix:=$ROSBAG_ADDRESS topics:="$TOPICS"