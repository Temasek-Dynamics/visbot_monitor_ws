#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

printf '%*s\n' "$(tput cols)" '' | tr ' ' '-'
echo -e "\033[1;33mIMPORTANT:\033[0m"
echo -e "\033[1;33mMake sure the autolaunch script has fully completed before proceeding to kill the nodes.\033[0m"
printf '%*s\n' "$(tput cols)" '' | tr ' ' '-'
sleep 1
echo "Swarm Options:"
echo "1) owl2 & owl3"
echo "2) owl2"
echo "3) owl3"
echo "0) owl3_factory"
read -p "Enter swarm selection (1, 2, 3, 0): " choice

case "$choice" in
    1) selection="owl2_and_owl3" ;;
    2) selection="owl2" ;;
    3) selection="owl3" ;;
    0) selection="owl3_factory" ;;
    *)
    echo -e "\033[31mInvalid selection. Exiting...\033[0m"
    exit 1
    ;;
esac

export SWARM_SELECTION="$selection"

printf '%*s\n' "$(tput cols)" '' | tr ' ' '-'
echo -e "\033[1;33mKill ROS Nodes\033[0m"
sleep 1
echo
"$SCRIPT_DIR/visbot_setup/kill_rosnode.sh"
echo
printf '%*s\n' "$(tput cols)" '' | tr ' ' '-'

echo -e "\033[1;33mClean ROS Logs\033[0m"
read -p "Need to clean ROS logs? (y/n): " yn
case $yn in
    [Yy]* ) 
        echo
        "$SCRIPT_DIR/visbot_setup/clean_roslog.sh";;
    [Nn]* ) 
        echo
        echo -e "\033[33mROS log cleaning skipped.\033[0m";;
    * ) 
        echo "Invalid input, please enter 'y' or 'n'." 
        exit 1;;
esac
echo
printf '%*s\n' "$(tput cols)" '' | tr ' ' '-'

echo -e "\033[1;33mTime Synchronization\033[0m"
read -p "Need to synchronize time? (y/n): " yn
case $yn in
    [Yy]* ) 
        echo
        "$SCRIPT_DIR/visbot_setup/time_sync.sh";;
    [Nn]* ) 
        echo
        echo -e "\033[33mTime synchronization skipped.\033[0m";;
    * ) 
        echo "Invalid input, please enter 'y' or 'n'." 
        exit 1;;
esac
echo
printf '%*s\n' "$(tput cols)" '' | tr ' ' '-'

echo -e "\033[1;33mRestart Swarm\033[0m"
echo
"$SCRIPT_DIR/visbot_setup/restart_swarm.sh"
echo