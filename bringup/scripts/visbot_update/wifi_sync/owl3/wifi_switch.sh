#!/bin/bash

install_yq() {
  if ! command -v yq &> /dev/null; then
      echo "Required package 'yq' not found. Installing yq using snap..."
      sudo snap install yq
  fi
}

# Main script starts here
install_yq

echo "Please choose the operation:"
echo "1) ap2net"
echo "2) net2ap"
read -p "Enter your choice (1 or 2): " choice

if [ "$choice" -eq 1 ]; then
  wifi_yaml="ap2net.yaml"
elif [ "$choice" -eq 2 ]; then
  wifi_yaml="net2ap.yaml"
else
  echo "Invalid choice. Exiting."
  exit 1
fi

template="wifi_format.tmpl"

current_wifi_name=$(yq eval '.current_wifi_name' $wifi_yaml)
current_wifi_pwd=$(yq eval '.current_wifi_pwd' $wifi_yaml)
current_gateway=$(yq eval '.current_gateway' $wifi_yaml)
current_ips=($(yq eval '.current_ips[]' $wifi_yaml))

new_wifi_name=$(yq eval '.new_wifi_name' $wifi_yaml)
new_wifi_pwd=$(yq eval '.new_wifi_pwd' $wifi_yaml)
new_gateway=$(yq eval '.new_gateway' $wifi_yaml)
new_ips=($(yq eval '.new_ips[]' $wifi_yaml))

export current_wifi_name current_wifi_pwd current_gateway
export new_wifi_name new_wifi_pwd new_gateway

for i in "${!current_ips[@]}"; do
  current_ip=${current_ips[$i]}
  new_ip=${new_ips[$i]}

  echo
  echo "Processing $current_ip -> $new_ip"
  export current_ip
  export new_ip
  envsubst < $template > ./wifi_cfg.sh
  scp ./wifi_cfg.sh visbot@$current_ip:/config/etc/wifi_cfg.sh > /dev/null 2>&1

# ##########################################################
# Restart the Wi-Fi following onboarding auto-launch scripts
# ##########################################################
#   ssh -T visbot@$current_ip <<'EOF' | grep -v 'Welcome to' | grep -v '*' | grep -v '__' | grep -v '\\' | sed '/^\s*$/d'
#   if [ -f /config/etc/wifi_cfg.sh ]; then
#     source /config/etc/wifi_cfg.sh
#   else
#     source /rootfs_app/deploy/wifi_cfg.sh
#   fi

#   if [ ! -f /config/etc/no_start_wifi_ap ]; then
#     cd /rootfs_app/sbin/
#     nohup ./wifi.sh > /dev/null 2>&1 &
#     echo "Wi-Fi is being restarted on $HOSTNAME"
#   fi
# EOF

  ssh -T visbot@$current_ip <<'EOF' | grep -v 'Welcome to' | grep -v '*' | grep -v '__' | grep -v '\\' | sed '/^\s*$/d'
  ip_address=$(hostname -I | awk '{print $1}')
  echo -e "\033[32mRebooting $HOSTNAME: $ip_address...\033[0m"
  sleep 1
  sudo reboot
EOF

  unset current_ip new_ip
  rm ./wifi_cfg.sh

done

echo
echo -n "Please manually connect the GSC to "
tput setaf 2
tput bold
echo -n "$new_wifi_name"
tput sgr0
echo "."

unset current_wifi_name current_wifi_pwd current_gateway current_ip
unset new_wifi_name new_wifi_pwd new_gateway new_ip