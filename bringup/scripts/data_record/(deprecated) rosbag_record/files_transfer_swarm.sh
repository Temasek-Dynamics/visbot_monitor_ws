#!/bin/bash

# Please update the following .json to match your setup
config_file="files_transfer_swarm.yaml"

install_yq() {
    if ! command -v yq &> /dev/null; then
        echo "Required package 'yq' not found. Installing yq using snap..."
        sudo snap install yq
    fi
}

# Function to install jq if not already installed
install_jq() {
    if ! command -v jq &> /dev/null; then
        echo "Required package 'jq' not found. Installing yq..."
        sudo apt-get update
        sudo apt-get install -y jq
    else
        echo "Required packages are installed. Running the script..."
    fi
}

install_yq
install_jq

if [ ! -f "$config_file" ]; then
    echo "Config file $config_file not found in the current directory!"
    exit 1
fi

test_name=$(yq eval '.test_name' $config_file)
test_date=$(yq eval '.test_date' $config_file)
drones=$(yq eval '.drones' $config_file -o=json | jq -c '.[]')
successful_drones=()

for drone in $drones; do
    drone_id=$(echo $drone | jq -r '.id')
    start_log=$(echo $drone | jq -r '.start_log')
    end_log=$(echo $drone | jq -r '.end_log')

    ./files_transfer.sh "$test_name" "$test_date" "$drone_id" "$start_log" "$end_log"

    if [ $? -eq 0 ]; then
        successful_drones+=($drone_id)
    else
        echo "Failed to transfer files from drone ID $drone_id"
    fi
done

total_drones=$(yq e '.drones | length' "$config_file")
if [ ${#successful_drones[@]} -eq $total_drones ]; then
    echo "All logs and bags are transferred successfully."
else
    if [ ${#successful_drones[@]} -gt 0 ]; then
        echo "Logs and bags are transferred successfully for drone IDs: ${successful_drones[@]}"
    else
        echo "No successful transfers."
    fi
fi