#!/bin/bash

# File transfer for one drone, requires test date, drone ID, start log number, and end log number.
# read -p "Please name the data folder: " test_name
# read -p "Please enter the test date (format: %Y-%m-%d): " test_date
# read -p "Please enter the drone ID: " drone_id
# read -p "Please enter the start log number: " start_log
# read -p "Please enter the end log number: " end_log

# For test only
# test_name=Outdoor-Test
# test_date=2024-06-28
# drone_id=2
# start_log=159
# end_log=165

# For file_transfer_swarm.sh
if [ "$#" -ne 5 ]; then
    echo "Usage: $0 <test_name> <test_date> <drone_id> <start_log> <end_log>"
    exit 1
fi
test_name=$1
test_date=$2
drone_id=$3
start_log=$4
end_log=$5

# File transfer start here!
remote_ip="192.168.2.2${drone_id}"
echo "Running rosbag_fix.sh on remote server..."
# Remote processing
ssh "visbot@${remote_ip}" << ENDSSH
cd ~/UserData/scripts
./rosbag_fix.sh ${test_date}
ENDSSH

# Check if the SSH command was successful
if [ $? -ne 0 ]; then
    echo "Failed to run rosbag_fix.sh on remote server."
    exit 1
fi

BAG_ADDRESS="${HOME}/visbot_bag/multi-drones/${test_date}-${test_name}/owl_${drone_id}/bags"
LOG_ADDRESS="${HOME}/visbot_bag/multi-drones/${test_date}-${test_name}/owl_${drone_id}/logs"
mkdir -p $BAG_ADDRESS
mkdir -p $LOG_ADDRESS

echo "Transferring files..."

# Transfer log files
for log_num in $(seq ${start_log} ${end_log}); do
    scp -r "visbot@${remote_ip}:~/log/log${log_num}" $LOG_ADDRESS
done

# Transfer bags files
scp "visbot@${remote_ip}:~/UserData/bags/${test_date}/*.bag" $BAG_ADDRESS

echo "File transfer complete."