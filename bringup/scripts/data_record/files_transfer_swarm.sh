#!/bin/bash

# Please update the following .yaml to match your setup
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
swarm_yaml="$SCRIPT_DIR/files_transfer_swarm.yaml"

install_yq() {
    if ! command -v yq &> /dev/null; then
        echo "Required package 'yq' not found. Installing yq using snap..."
        sudo snap install yq
    fi
}

install_yq

if [ ! -f "$swarm_yaml" ]; then
    echo "Config file $swarm_yaml not found in the current directory!"
    exit 1
fi

dir_name=$(yq eval '.dir_name' $swarm_yaml)
test_name=$(yq eval '.test_name' $swarm_yaml)
test_date=$(yq eval '.test_date' $swarm_yaml)
drone_ips=$(yq eval '.drone_ips[]' "$swarm_yaml")

echo
successful_drones=()

for drone_ip in $drone_ips; do
    if ssh -n -f "visbot@$drone_ip" "exit" > /dev/null 2>&1; then
        echo
        echo -e "\033[32m$drone_ip\033[0m: SSH Connected, Transferring files..."
        ./files_transfer.sh "$dir_name" "$test_name" "$test_date" "$drone_ip"

        if [ $? -eq 0 ]; then
            successful_drones+=("$drone_ip")
        else
            echo "Failed to transfer files from drone IP $drone_ip"
        fi
    else
        echo -e "\033[31m$drone_ip\033[0m: SSH Connection Failed"
    fi
done

echo
total_drones=$(yq eval '.drone_ips | length' "$swarm_yaml")
if [ ${#successful_drones[@]} -eq $total_drones ]; then
    echo -e "\033[32mAll drones have successfully transferred logs and bags.\033[0m"
else
    if [ ${#successful_drones[@]} -gt 0 ]; then
        echo -e "Following drones have successfully transferred logs and bags:\n\033[33m${successful_drones[@]}\033[0m"
    else
        echo "No successful transfers."
    fi
fi