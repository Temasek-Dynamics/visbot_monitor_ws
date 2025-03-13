#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

printf '%*s\n' "$(tput cols)" '' | tr ' ' '-'
echo -e "\033[1;33mQGroundControl\033[0m"
echo

QGC_PATH=~/QGroundControl.AppImage
if [ ! -f "$QGC_PATH" ]; then
    echo -e "\033[31mError: QGroundControl.AppImage not found at $QGC_PATH\033[0m"
    exit 1
fi

if ! pgrep -x "QGroundControl" > /dev/null; then
    "$QGC_PATH" >/dev/null 2>&1 &
fi

sleep 2

read -p $'\033[38;5;198mConnection between GSC and Swarm Established in QGroundControl? (y/n): \033[0m' yn
echo
printf '%*s\n' "$(tput cols)" '' | tr ' ' '-'

case $yn in
    [Yy]* ) 
        echo -e "\033[1;33mSwarm GUI\033[0m"
        "$SCRIPT_DIR/visbot_setup/swarm_gui.sh"
        ;;
    [Nn]* ) exit;;
    * ) echo "Invalid Input";;
esac