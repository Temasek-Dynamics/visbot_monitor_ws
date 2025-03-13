#!/bin/bash

install_yq() {
  if ! command -v yq &> /dev/null; then
      echo "Required package 'yq' not found. Installing yq using snap..."
      sudo snap install yq
  fi
}

# Main script starts here
install_yq

echo "Please choose the router:"
echo "1) visbot_ap"
echo "2) visbot_net_5GHz"
read -p "Enter your choice (1 or 2): " choice

if [ "$choice" -eq 1 ]; then
  wifi_yaml="ap2net.yaml"
elif [ "$choice" -eq 2 ]; then
  wifi_yaml="net2ap.yaml"
else
  echo "Invalid choice. Exiting."
  exit 1
fi

new_wifi_name=$(yq eval '.new_wifi_name' $wifi_yaml)
new_wifi_pwd=$(yq eval '.new_wifi_pwd' $wifi_yaml)

if nmcli dev wifi connect "$new_wifi_name" password "$new_wifi_pwd" > /dev/null 2>&1; then
  echo -n "GSC has successfully connected to "
  tput setaf 2
  tput bold
  echo -n "$new_wifi_name"
  tput sgr0
  echo "."
else
  echo -n "Failed to connect to "
  tput setaf 1
  tput bold
  echo -n "$new_wifi_name"
  tput sgr0
  echo "."
fi

echo

unreachable_ips=()
for new_ip in "${new_ips[@]}"; do
  count=$((count + 1))
  printf "Pinging $new_ip..."

  if ping -c 4 $new_ip > /dev/null 2>&1; then
    tput setaf 2
    echo "Connected"
    tput sgr0
  else
    tput setaf 1
    echo "Failed"
    tput sgr0
    unreachable_ips+=($new_ip)
  fi
done

if [ ${#unreachable_ips[@]} -eq 0 ]; then
  tput setaf 2
  tput bold
  echo "All IPs are connected"
  tput sgr0
else
  echo
  echo "Unreachable IPs:"
  tput setaf 1
  tput bold
  for ip in "${unreachable_ips[@]}"; do
    echo "$ip"
  done
  tput sgr0
fi