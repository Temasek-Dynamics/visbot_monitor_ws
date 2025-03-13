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

install_yq

printf '%*s\n' "$(tput cols)" '' | tr ' ' '-'
echo
echo -e "\033[1;33mIMPORTANT: Make sure \033[1;31mROSCORE KILLED \033[1;33mbefore cleaning roslogs.\033[0m"
echo
printf '%*s\n' "$(tput cols)" '' | tr ' ' '-'
echo

sleep 1

read -p "Start ROS log cleaning? (y/n): " yn
echo

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

for ip in $ips; do
    if ssh -n -f "visbot@$ip" "exit" > /dev/null 2>&1; then
        log_size=$(ssh "visbot@$ip" "sudo du -sk /root/.ros/log | cut -f1")

        # 1000000 KB = 1 GB
        if [ "$log_size" -lt 1000000 ]; then
            echo -e "\033[32m$ip\033[0m: /root/.ros/log size < 1GB. Cleanup skipped"
        else
            ssh "visbot@$ip" "sudo su -c 'rm -rf /root/.ros/log/*'"

            log_contents=$(ssh -n "visbot@$ip" "sudo ls -A /root/.ros/log")
            if [ -z "$log_contents" ]; then
                echo -e "\033[32m$ip\033[0m: /root/.ros/log cleaned"
            else
                echo -e "\033[31m$ip\033[0m: /root/.ros/log is not empty. Remaining files:"
                echo "$log_contents"
                echo
            fi
        fi
    else
        echo -e "\033[31m$ip\033[0m: SSH connection failed"
    fi
done

