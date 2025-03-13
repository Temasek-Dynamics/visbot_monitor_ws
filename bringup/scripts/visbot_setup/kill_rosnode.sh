#!/bin/bash

install_yq() {
    if ! command -v yq &> /dev/null; then
        echo "Required package 'yq' not found. Installing yq using snap..."
        sudo snap install yq
    fi
}

read_ips() {
  local selection="$1"
  yq eval ".${selection}.drone_ips[]" "$yaml_file"
}

is_killed() {
    local ip="$1"
    printf "\033[32m$ip\033[0m: ROSCORE... "
    ssh -n "visbot@$ip" "source /opt/ros/noetic/setup.bash && rosnode ping /rosout -c 1" > /dev/null 2>&1

    if [ $? -eq 0 ]; then
        printf "\033[33mRunning\033[0m\n"
        echo "--------------------------------"
        return 1
    else
        printf "\033[32mKilled\033[0m\n"
        return 0
    fi
}

install_yq

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

    echo
else
    selection="$SWARM_SELECTION"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
yaml_file="$SCRIPT_DIR/swarm_ip.yaml"
ips=$(read_ips "$selection")

echo -e "Selected Swarm: \033[32m$selection\033[0m"
echo
echo "Drone IPs:"
echo "$ips"
echo

echo -e "\033[36mKilling All Drones' Nodes...\033[0m"

successful_ips=()
is_all_connected=true

for ip in $ips; do
    if ssh -n -f "visbot@$ip" "exit" > /dev/null 2>&1; then
        ssh -n "visbot@$ip" "
        source /opt/ros/noetic/setup.bash && \
        rosnode kill -a && \
        sudo pkill -f roslaunch && \
        sudo pkill -f rosmaster && \
        sudo pkill -f rosout
        " > /dev/null 2>&1

        successful_ips+=("$ip")
        echo -e "\033[32m$ip\033[0m: \033[36mAll Nodes killed\033[0m"
    else
        # echo -e "\033[31m$ip\033[0m: \033[35mSSH connection failed\033[0m"
        echo -e "\033[31m$ip\033[0m: SSH connection failed"
        is_all_connected=false
    fi
done

echo
echo "ROS Status:"
ips="${successful_ips[@]}"

all_killed=false
while [ "$all_killed" = false ]; do
    all_killed=true
    for ip in $ips; do
        if ! is_killed "$ip"; then
            all_killed=false
            break
        fi
    done

    if [ "$all_killed" = false ]; then
        sleep 1
    fi
done


echo
if [ "$is_all_connected" = true ]; then
    echo -e "\033[1;32mROS killed on ALL drones.\033[0m"
else
    echo -e "\033[1;33mROS killed on CONNECTED drones.\033[0m"
fi