#!/bin/bash

SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )/.."
source /opt/ros/noetic/setup.bash
source $SCRIPT_DIR/../../devel/setup.bash

drone_num=12

echo
echo -e "Drone Number: \033[1;32m$drone_num\033[0m"
echo

echo -e "\033[36mROS Output\033[0m"
printf '%*s\n' "$(tput cols)" '' | tr ' ' '-'

if ! rosnode list 2>/dev/null | grep -q "/commander_ros_proxy"; then
    roslaunch ros_proxy ros_proxy_commander_gui.launch &
    sleep 3
fi

roslaunch swarm_gui swarm_gui.launch drone_num:=$drone_num