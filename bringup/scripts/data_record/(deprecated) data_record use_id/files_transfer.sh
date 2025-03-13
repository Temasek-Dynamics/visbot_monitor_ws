#!/bin/bash

# For file_transfer_swarm.sh
if [ "$#" -ne 3 ]; then
    echo "Usage: $0 <test_name> <test_date> <drone_id>"
    exit 1
fi
test_name=$1
test_date=$2
drone_id=$3

# File transfer start here!
remote_ip="192.168.2.2${drone_id}"
echo "Running rosbag_fix.sh on remote server..."
ssh "visbot@${remote_ip}" << ENDSSH
cd ~/UserData/scripts/data_record
./fix_rosbag.sh ${test_date}
ENDSSH

# Check if the SSH command was successful
if [ $? -ne 0 ]; then
    echo "Failed to run rosbag_fix.sh on remote server."
    exit 1
fi

DATA_ADDRESS="${HOME}/visbot_experiment/multi-drones/${test_date}-${test_name}/owl_${drone_id}"
mkdir -p $DATA_ADDRESS

echo "Transferring files..."

# Transfer files
scp -r "visbot@${remote_ip}:~/UserData/experiments/${test_date}/*" $DATA_ADDRESS

echo "File transfer complete."
echo