#!/bin/bash

printf '%*s\n' "$(tput cols)" '' | tr ' ' '-'
echo -e "\033[1;33mDrone Visualization\033[0m"
echo

echo "Visualization Options:"
echo "2) SAFMC"
echo "1) Swarm"
echo "0) Single"
read -p "Enter your choice (1 or 0): " choice

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ "$choice" = "2" ]; then
    "$SCRIPT_DIR/visbot_visualization/safmc_visualization.sh"
elif [ "$choice" = "1" ]; then
    "$SCRIPT_DIR/visbot_visualization/swarm_visualization.sh"
elif [ "$choice" = "0" ]; then
    "$SCRIPT_DIR/visbot_visualization/single_visualization.sh"
else
    echo "Invalid choice. Exiting."
    exit 1
fi