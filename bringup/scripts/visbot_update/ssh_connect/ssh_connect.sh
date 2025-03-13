#!/bin/bash

install_yq() {
    if ! command -v yq &> /dev/null; then
        echo "Required package 'yq' not found. Installing yq using snap..."
        sudo snap install yq
    fi
}

install_sshpass() {
  if ! command -v sshpass &> /dev/null; then
    echo "sshpass not found. Installing sshpass..."
    sudo apt-get update && sudo apt-get install -y sshpass
    
    if [ $? -eq 0 ]; then
      echo "sshpass successfully installed."
    else
      echo -e "\033[31mFailed to install sshpass. Exiting...\033[0m"
      exit 1
    fi
  fi
}

read_ips() {
  local selection="$1"
  yq eval ".${selection}.drone_ips[]" "$yaml_file"
}


remove_old_key() {
  local ip="$1"
  
  echo "Checking for existing SSH key in known_hosts for $ip..."

  ssh_output=$(ssh -o BatchMode=yes -o StrictHostKeyChecking=yes "visbot@$ip" "exit" 2>&1)
  ssh_exit_code=$?

  if [ $ssh_exit_code -eq 0 ]; then
    echo -e "\033[32mSuccessfully connected to $ip. No host key conflict.\033[0m"
    return 0 
  
  elif echo "$ssh_output" | grep -q "REMOTE HOST IDENTIFICATION HAS CHANGED"; then
    echo "Host key conflict detected for $ip. Removing old SSH key from known_hosts..."
    ssh-keygen -R "$ip" &>/dev/null
    if [ $? -eq 0 ]; then
      echo "Old SSH key for $ip removed from known_hosts."
      return 2
    else
      echo -e "\033[31mFailed to remove SSH key for $ip from known_hosts.\033[0m"
      return 1
    fi
  
  elif echo "$ssh_output" | grep -q "No ECDSA host key is known for"; then
    echo "First connection and ready to update known_hosts..."
    return 2
  
  else
    echo -e "\033[31mError occurred while connecting to $ip. $ssh_output\033[0m"
    return 1
  fi
}

update_known_hosts() {
  local ip="$1"
  
  echo "Updating known_hosts for $ip..."
  sshpass -p "$password" ssh -o StrictHostKeyChecking=no "visbot@$ip" "exit" &>/dev/null

  if [ $? -eq 0 ]; then
    echo "$ip has been added to known_hosts."
    return 0
  else
    echo -e "\033[31mFailed to update known_hosts for $ip.\033[0m"
    return 1
  fi
}

copy_ssh_key() {
  local ip="$1"
  
  echo "Copying SSH key to $ip..."
  sshpass_output=$(sshpass -p "$password" ssh-copy-id "visbot@$ip" 2>&1)
  # local max_length=$(echo "$sshpass_output" | awk '{ print length }' | sort -nr | head -n 1)
  # local border=$(printf '#%.0s' $(seq 1 $max_length))

  echo "Notification from 'sshpass':"
  # echo "$border"
  printf '%*s\n' "${COLUMNS:-80}" '' | tr ' ' '#'
  echo "$sshpass_output" | grep -v '^$'
  # echo "$border"
  printf '%*s\n' "${COLUMNS:-80}" '' | tr ' ' '#'

  if [ $? -eq 0 ]; then
    echo -e "\033[32m$ip, SSH Connected.\033[0m"
    return 0
  else
    echo -e "\033[31mFailed to copy SSH key to $ip.\033[0m"
    return 1
  fi
}

connect_and_copy_ssh_key() {
  local ip="$1"

  remove_old_key "$ip"
  local remove_key_status=$?
  
  if [ $remove_key_status -eq 0 ] || [ $remove_key_status -eq 1 ]; then
    return
  fi
  
  if [ $remove_key_status -eq 2 ]; then
    if ! update_known_hosts "$ip"; then
      return
    fi
  fi

  if ! copy_ssh_key "$ip"; then
    echo -e "\033[31mFailed to copy SSH key to $ip.\033[0m"
  fi
}

# Main script starts here
install_yq
install_sshpass

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
password="visbot"
ips=$(read_ips "$selection")

echo -e "Selected Swarm: \033[32m$selection\033[0m"
echo
echo "Drone IPs:"
echo "$ips"
echo

for ip in $ips; do
  connect_and_copy_ssh_key "$ip"
  echo
done

echo "All Done."