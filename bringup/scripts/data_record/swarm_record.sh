#!/bin/bash

install_yq() {
    if ! command -v yq &> /dev/null; then
        echo "Required package 'yq' not found. Installing yq using snap..."
        sudo snap install yq
    fi
}

read_ips() {
  local selection="$1"
  yq eval ".${selection}.drone_ips[]" "$swarm_yaml"
}

start_record() {
  local exp_num=$1
  for drone_ip in $drone_ips; do
    echo
    echo "Connecting to $drone_ip..."
    
    ssh_output=$(ssh -o ConnectTimeout=1 "visbot@$drone_ip" 'printf "DONE"' 2>&1)
    
    if echo "$ssh_output" | grep -q 'DONE'; then
      echo -e "\033[32mConnected to $drone_ip\033[0m. Running data_record.sh..."
      if [ -n "$exp_num" ]; then
        ssh "visbot@$drone_ip" "cd $script_dir && nohup ./$script_launch $exp_num" &
      else
        ssh "visbot@$drone_ip" "cd $script_dir && nohup ./$script_launch" &
      fi
      sleep 1
    else
      echo -e "\033[31mFailed to connect to $drone_ip\033[0m. $ssh_output"
    fi
  done
}

stop_record() {
  for drone_ip in $drone_ips; do
    echo
    echo "Connecting to $drone_ip to stop data_record.sh..."

    ssh_output=$(ssh -o ConnectTimeout=1 "visbot@$drone_ip" 'echo "DONE"' 2>&1)

    if echo "$ssh_output" | grep -q 'DONE'; then
      echo -e "\033[32mConnected to $drone_ip\033[0m. Stopping data_record.sh... \c"

      ssh_killed=$(ssh "visbot@$drone_ip" "cd $script_dir && nohup ./$script_kill" 2>&1)
      if echo "$ssh_killed" | grep -q 'killed'; then
        printf "\033[32mKilled\033[0m\n"
      else
        printf "\033[31mFailed to stop\033[0m\n"
      fi
    else
      echo -e "\033[31mFailed to connect to $drone_ip\033[0m. $ssh_output"
    fi
  done
}

# Main script starts here
install_yq

exp_num=$1
if [ -n "$exp_num" ]; then
  echo "Experiment number: $exp_num"
fi

if [ -z "$SWARM_SELECTION" ]; then
    echo "Swarm Options:"
    echo "1) owl2 & owl3"
    echo "2) owl2"
    echo "3) owl3"
    echo "0) owl3_factory"
    read -p "Enter swarm selection (1, 2, 3, 0): " choice

    case "$choice" in
      1) selection="owl2_and_owl3" ;;
      2) selection="owl2" ;;
      3) selection="owl3" ;;
      0) selection="owl3_factory" ;;
      *)
        echo -e "\033[31mInvalid selection. Exiting...\033[0m"
        exit 1
        ;;
    esac
else
    selection="$SWARM_SELECTION"
fi

echo

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
swarm_yaml="$SCRIPT_DIR/swarm_ip.yaml"
script_dir="~/ros_ws/src/visbot_scripts/data_record"
script_launch="data_record.sh"
script_kill="data_record_kill.sh"
drone_ips=$(read_ips "$selection")

echo -e "Selected Swarm: \033[32m$selection\033[0m"
echo
echo -e "Drone IPs: \n$drone_ips"
start_record "$exp_num"
sleep 3

echo
while true; do
  read -p "Enter 'END' to stop recording: " cmd
  if [[ "$cmd" == "END" ]]; then
    stop_record
    break
  fi
done