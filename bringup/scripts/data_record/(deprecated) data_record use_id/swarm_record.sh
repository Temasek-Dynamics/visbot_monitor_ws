#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
swarm_config="$SCRIPT_DIR/swarm_id.yaml"

read_drone_ids() {
  local yaml_file=$1
  drone_ids=$(grep -A 1000 'drone_ids:' "$yaml_file" | grep -oP '\-\s*\K[^\s]+' | tr '\n' ' ')
  echo "$drone_ids"
}

start_record() {
  local exp_num=$1
  for drone_id in $drone_ids; do
    remote_ip="192.168.2.2$((10#$drone_id))"
    echo "Running data_record.sh on remote server $remote_ip..."
    if [ -n "$exp_num" ]; then
      ssh "visbot@$remote_ip" "cd ~/UserData/scripts/data_record && nohup ./data_record.sh $exp_num" &
    else
      ssh "visbot@$remote_ip" "cd ~/UserData/scripts/data_record && nohup ./data_record.sh" &
    fi
    sleep 1
    echo
    echo $remote_ip >> active_drones.txt
  done
}

stop_record() {
  for drone_id in $drone_ids; do
    remote_ip="192.168.2.2$((10#$drone_id))"
    echo
    echo "Stopping data_record.sh on remote server $remote_ip..."
    ssh "visbot@$remote_ip" "cd ~/UserData/scripts/data_record && nohup ./data_record_kill.sh"
    if [ $? -ne 0 ]; then
      echo "Failed to stop data_record.sh on $remote_ip"
    else
      echo "data_record.sh on $remote_ip is killed"
    fi
  done
  rm active_drones.txt
}

exp_num=$1
if [ -n "$exp_num" ]; then
  echo "Experiment number: $exp_num"
fi

drone_ids=$(read_drone_ids $swarm_config)
echo "Drone IDs: $drone_ids"
echo
start_record "$exp_num"
sleep 3

while true; do
  read -p "Enter 'END' to stop recording: " cmd
  if [[ "$cmd" == "END" ]]; then
    stop_record
    break
  fi
done

# ############################################################################################################
# Below is the original code for reference
# ############################################################################################################
# #!/bin/bash

# swarm_config="swarm_id.yaml"

# read_drone_ids() {
#   local yaml_file=$1
#   drone_ids=$(grep -A 1000 'drone_ids:' "$yaml_file" | grep -oP '\-\s*\K[^\s]+' | tr '\n' ' ')
#   echo "$drone_ids"
# }

# start_record() {
#   for drone_id in $drone_ids; do
#     remote_ip="192.168.2.2$((10#$drone_id))"
#     echo "Running data_record.sh on remote server $remote_ip..."
#     ssh "visbot@$remote_ip" "cd ~/UserData/scripts/data_record && nohup ./data_record.sh" &
#     sleep 1
#     echo
#     echo $remote_ip >> active_drones.txt
#   done
# }

# stop_record() {
#   for drone_id in $drone_ids; do
#     remote_ip="192.168.2.2$((10#$drone_id))"
#     echo
#     echo "Stopping data_record.sh on remote server $remote_ip..."
#     ssh "visbot@$remote_ip" "cd ~/UserData/scripts/data_record && nohup ./data_record_kill.sh"
#     if [ $? -ne 0 ]; then
#       echo "Failed to stop data_record.sh on $remote_ip"
#     else
#       echo "data_record.sh stopped successfully on $remote_ip"
#     fi
#   done
#   rm active_drones.txt
# }

# drone_ids=$(read_drone_ids $swarm_config)
# echo "Drone IDs: $drone_ids"
# echo
# start_record
# sleep 3

# while true; do
#   read -p "Enter 'END' to stop recording: " cmd
#   if [[ "$cmd" == "END" ]]; then
#     stop_record
#     break
#   fi
# done