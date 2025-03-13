#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# printf '%*s\n' "$(tput cols)" '' | tr ' ' '-'
# echo -e "\033[1;33mTime Synchronization\033[0m"
# "$SCRIPT_DIR/visbot_setup/time_sync.sh"
# echo

printf '%*s\n' "$(tput cols)" '' | tr ' ' '-'
echo -e "\033[1;33mPose Initialization\033[0m"
echo
read -p "Need to initialize swarm pose? (y/n): " yn
case $yn in
    [Yy]* ) 
        echo
        "$SCRIPT_DIR/visbot_setup/init_pose.sh";;
    [Nn]* ) 
        echo
        echo -e "\033[33mPose initialization skipped.\033[0m";;
    * ) 
        echo "Invalid input, please enter 'y' or 'n'." 
        exit 1;;
esac
echo

printf '%*s\n' "$(tput cols)" '' | tr ' ' '-'
echo
echo -e "\033[1;33mPlease Launch the Swarm and Ensure All Drones have Taken Off.\033[0m"
echo
printf '%*s\n' "$(tput cols)" '' | tr ' ' '-'

sleep 1

echo
read -p $'\033[38;5;198mSwarm Takeoff and Ready to Start Mission? (y/n): \033[0m' yn
echo
printf '%*s\n' "$(tput cols)" '' | tr ' ' '-'
case $yn in
    [Yy]* ) 
        echo -e "\033[1;33mStart Mission\033[0m"
        echo
        "$SCRIPT_DIR/visbot_setup/mission_loop.sh";;
    [Nn]* ) exit;;
    * ) echo "Invalid Input";;
esac