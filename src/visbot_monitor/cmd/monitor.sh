#!/bin/bash

pkill -f "rviz"
pkill -f "QGroundControl"

sleep 1

# Check if the user provided the correct argument
if [ $# -ne 1 ] || ([ "$1" != "0" ] && [ "$1" != "1" ]); then
    echo "Usage: $0 <0|1>"
    echo "0: Do not record rosbag"
    echo "1: Record rosbag"
    exit 1
fi

# rviz
source $HOME/visbot_ws/devel/setup.bash
echo "RViz is starting..."
roslaunch visbot_monitor rviz.launch &
# nohup roslaunch visbot_monitor rviz.launch > /dev/null 2>&1 &

# QGroundControl
echo "QGroundControl is starting..."
$HOME/QGroundControl.AppImage &
# nohup $HOME/QGroundControl.AppImage > /dev/null 2>&1 &

# 5 is a good number to wait for the windows to open
sleep 5

# Obtain window IDs for RViz and QGroundControl
rviz_window_id=$(wmctrl -l | grep "RViz" | awk '{print $1}')
qgc_window_id=$(wmctrl -l | grep "QGroundControl" | awk '{print $1}')

# Print error msg if no window found
if [ -z "$rviz_window_id" ] || [ -z "$qgc_window_id" ]; then
    echo "Failed to find window IDs for RViz and/or QGroundControl."
    exit 1
fi

# Move qgc 2 Right
# wmctrl -r "QGroundControl" -e 0,960,0,-1,-1
xdotool key --window $qgc_window_id Super+Right

# Move rviz 2 Left
# wmctrl -r "RViz" -e 0,0,0,-1,-1
xdotool key --window $rviz_window_id Super+Left
# for i in {1}; do
#     xdotool key --window $rviz_window_id Super+Left
#     sleep 0.5
# done

sleep 3

# Start recording rosbag if the argument is 1
if [ "$1" == "1" ]; then
    echo "Recording rosbag..."
    rosbag record -o $HOME/visbot_ws/src/visbot_monitor/bag/rosbag_$(date +%Y-%m-%d_%H-%M-%S) $(cat topics.txt)
fi

# Wait for user input to exit (Ctrl + C)
echo "Press Ctrl + C to exit."
trap "exit" INT
while :
do
    sleep 3
done