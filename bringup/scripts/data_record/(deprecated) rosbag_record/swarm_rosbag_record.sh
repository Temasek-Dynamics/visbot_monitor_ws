#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
swarm_config="$SCRIPT_DIR/swarm_id.yaml"

read_drone_ids() {
  local yaml_file=$1
  drone_ids=$(grep -A 1000 'drone_ids:' "$yaml_file" | grep -oP '\-\s*\K[^\s]+' | tr '\n' ' ')
  echo "$drone_ids"
}

start_rosbag_record() {
  for drone_id in $drone_ids; do
    remote_ip="192.168.2.2$((10#$drone_id))"
    echo "Running rosbag_record.sh on remote server $remote_ip..."
    # ssh "visbot@$remote_ip" "cd ~/UserData/scripts && nohup ./rosbag_record.sh > ~/UserData/scripts/rosbag_record.log 2>&1 &" &
    ssh "visbot@$remote_ip" "cd ~/UserData/scripts && nohup ./rosbag_record.sh" 
    if [ $? -ne 0 ]; then
      echo "Failed to start rosbag_record.sh on $remote_ip"
    else
      echo "rosbag_record.sh started successfully on $remote_ip"
      echo
    fi
    echo $remote_ip >> active_drones.txt
  done
}

stop_rosbag_record() {
  for drone_id in $drone_ids; do
    remote_ip="192.168.2.2$((10#$drone_id))"
    echo "Stopping rosbag_record_node on remote server $remote_ip..."
    ssh "visbot@$remote_ip" "source /opt/ros/noetic/setup.bash && rosnode kill /rosbag_record_node"
    if [ $? -ne 0 ]; then
      echo "Failed to stop rosbag_record_node on $remote_ip"
    else
      echo "rosbag_record_node stopped successfully on $remote_ip"
    fi
  done
  rm active_drones.txt
}

# ####################################################################################################################################
# The performance of the following stop_rosbag_record() function to kill rosnode simultaneously on multiple drones is not guaranteed.
# The function can only worked occasionally (may caused by network connecting issues), and it is not recommended to use this function.
# ####################################################################################################################################
# stop_rosbag_record() {
#   for drone_id in $drone_ids; do
#     remote_ip="192.168.2.2$((10#$drone_id))"
#     echo "Stopping rosbag_record_node on remote server $remote_ip..."
#     ssh "visbot@$remote_ip" "cd ~/UserData/scripts && nohup ./rosbag_record_kill.sh > ~/UserData/scripts/rosbag_record_kill.log 2>&1 &" &
#     if [ $? -ne 0 ]; then
#       echo "Failed to stop rosbag_record_node on $remote_ip"
#     else
#       echo "rosbag_record_node stopped successfully on $remote_ip"
#     fi
#   done

#   wait

#   rm active_drones.txt
# }

drone_ids=$(read_drone_ids $swarm_config)
echo "Drone IDs: $drone_ids"
echo
start_rosbag_record

while true; do
  read -p "Enter 'END' to stop recording: " cmd
  if [[ "$cmd" == "END" ]]; then
    stop_rosbag_record
    break
  fi
done