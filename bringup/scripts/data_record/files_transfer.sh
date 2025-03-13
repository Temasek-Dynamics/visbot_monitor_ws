#!/bin/bash

# For file_transfer_swarm.sh
if [ "$#" -ne 4 ]; then
    echo "Usage: $0 <dir_name> <test_name> <test_date> <drone_ip>"
    exit 1
fi
dir_name=$1
test_name=$2
test_date=$3
drone_ip=$4

# File transfer start here!
echo -e "\033[1;33mRunning rosbag_fix.sh on remote server...\033[0m"
printf '%*s\n' "$(tput cols)" '' | tr ' ' '-'
ssh "visbot@${drone_ip}" << ENDSSH
cd ~/ros_ws/src/visbot_scripts/data_record
./fix_rosbag.sh ${test_date}
ENDSSH
echo

# Check if the SSH command was successful (depredated, as the failure check can not work as expected)
# if [ $? -ne 0 ]; then
#     echo "Failed to run rosbag_fix.sh on remote server."
#     exit 1
# fi

drone_id=${drone_ip##*.}
# echo "Drone ID: $drone_id"
data_dir="${HOME}/visbot_experiment/${dir_name}/${test_date}-${test_name}/owl_${drone_id}"
mkdir -p $data_dir

echo -e "\033[1;33mTransferring files from $drone_ip...\033[0m"

# Transfer files
scp -r "visbot@${drone_ip}:~/UserData/experiments/${test_date}/*" $data_dir
echo
echo -e "\033[1;33mFiles from $drone_ip transferred to $data_dir.\033[0m"
echo
printf '%*s\n' "$(tput cols)" '' | tr ' ' '-'