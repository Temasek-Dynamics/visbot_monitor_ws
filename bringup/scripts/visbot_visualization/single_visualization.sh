#!/bin/bash

# Modify the IP addresses manually according to your network configuration and the drone IP.
# export ROS_IP=192.168.2.101
# export ROS_HOSTNAME=192.168.2.101
# export ROS_MASTER_URI=http://192.168.2.20:11311

# Get the IP address of the GCS automatically
read -p "Enter the last two digits of the selected drone's IP address (e.g., 20, 30): " DRONE_IP
GCS_IP=$(hostname -I | awk '{print $1}')
echo "Ground station IP: $GCS_IP"
echo

export ROS_IP=$GCS_IP
export ROS_HOSTNAME=$GCS_IP
export ROS_MASTER_URI=http://192.168.2.${DRONE_IP}:11311

# echo "ROS_IP=$ROS_IP"
# echo "ROS_HOSTNAME=$ROS_HOSTNAME"
# echo "ROS_MASTER_URI=$ROS_MASTER_URI"

SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )/.."
# echo "SCRIPT_DIR=$SCRIPT_DIR"

source $SCRIPT_DIR/../../devel/setup.bash

if [[ $DRONE_IP -ge 20 && $DRONE_IP -le 23 ]]; then
    DEPTH_TOPIC="/stereo_cam/depth"
    DEPTH_INFO="/stereo_cam/depth_info"
elif [[ $DRONE_IP -ge 30 && $DRONE_IP -le 37 ]]; then
    DEPTH_TOPIC="/visbot_itof/depth"
    DEPTH_INFO="/visbot_itof/depth_info"
else
    echo "Invalid DRONE_IP range. Please enter a valid number between 20-23 or 30-37."
    exit 1
fi

roslaunch visbot_monitor depth_visualization.launch depth_topic:=$DEPTH_TOPIC depth_info:=$DEPTH_INFO > /dev/null 2>&1 &
roslaunch visbot_monitor single_visualization.launch