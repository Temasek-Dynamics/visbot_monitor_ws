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

is_restarted() {
    local ip="$1"
    ssh -n -f "visbot@$ip" "source /opt/ros/noetic/setup.bash; rosnode list" | grep "/ros_proxy" > /dev/null

    if [ $? -eq 0 ]; then
        echo -e "\033[32m$ip\033[0m: ROS Nodes Restarted"
        return 0
    else
        return 1
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

echo -e "\033[36mRestarting All Drones...\033[0m"

successful_ips=()
is_all_connected=true
for ip in $ips; do
    # echo "Restarting $ip..."
    if ssh -n -f "visbot@$ip" "exit" > /dev/null 2>&1; then
        ssh -n -f "visbot@$ip" "sudo \$HOME/bin/visquad.sh > /dev/null 2>&1 &"
        successful_ips+=("$ip")
        echo -e "\033[32m$ip\033[0m: ROS Nodes Restarting..."
    else
        echo -e "\033[31m$ip\033[0m: SSH connection failed"
        is_all_connected=false
    fi
done

echo
echo -e "\033[36mChecking Restart Status...\033[0m"
ips="${successful_ips[@]}"

# The Master Drone will take approximately 60 seconds to execute ~/bin/visquad.sh
sleep 36

ips_to_check=($ips)
while [ "${#ips_to_check[@]}" -gt 0 ]; do
    remaining_ips=()

    for ip in "${ips_to_check[@]}"; do
        if ! is_restarted "$ip"; then
            remaining_ips+=("$ip")
        fi
    done

    ips_to_check=("${remaining_ips[@]}")

    if [ "${#ips_to_check[@]}" -gt 0 ]; then
        sleep 1
    fi
done

# #################################################
# Monitor restart status of each drone individually
# #################################################
# echo
# all_restarted=false
# while [ "$all_restarted" = false ]; do
#     all_restarted=true
#     new_ips=()
#     for ip in $ips; do
#         if is_restarted "$ip"; then
#             echo "Drone $ip Restarted"
#         else
#             new_ips+=("$ip")
#             all_restarted=false
#         fi
#     done

#     ips=("${new_ips[@]}") 

#     if [ "$all_restarted" = false ]; then
#         sleep 3
#     fi
# done

echo
if [ "$is_all_connected" = true ]; then
    echo -e "\033[1;32mAll Drones Restarted. Ready to Fly.\033[0m" 
else
    echo -e "\033[1;33mConnected Drones Restarted. Ready to Fly.\033[0m"
fi