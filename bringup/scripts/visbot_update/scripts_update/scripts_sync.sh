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


update_bashrc_and_copy_scripts() {
  local ip="$1"

  echo "Copying visbot_scripts to $ip..."
  
  scp -r "$visbot_scripts_dir" "visbot@$ip:~/ros_ws/src/"
  # rsync -av "$visbot_scripts_dir" "visbot@$ip:~/ros_ws/src/"
  
  if [ $? -ne 0 ]; then
    echo "Failed to copy scripts to $ip."
    echo
    return 1
  fi

  echo
  # echo "Updating visbot commands to ~/.bashrc and setting execute permissions on $ip..."

  ssh "visbot@$ip" "bash -c '{
    sed -i \"/# visbot commands/d\" ~/.bashrc
    sed -i \"/alias visbot_restart=/d\" ~/.bashrc
    sed -i \"/alias visbot_record=/d\" ~/.bashrc
    sed -i \"/alias visbot_record_kill=/d\" ~/.bashrc
    sed -i \"/alias visbot_rosbag_record=/d\" ~/.bashrc
    sed -i \"/alias visbot_rosbag_record_kill=/d\" ~/.bashrc
    sed -i \"/alias visbot_raw_record=/d\" ~/.bashrc

    if [ -n \"\$(tail -n 1 ~/.bashrc)\" ]; then
        echo \"\" >> ~/.bashrc
    fi

    echo \"# visbot commands\" >> ~/.bashrc
    echo \"alias visbot_restart=\\\"sudo /home/visbot/bin/visquad.sh\\\"\" >> ~/.bashrc
    echo \"alias visbot_record=\\\"/home/visbot/ros_ws/src/visbot_scripts/data_record/data_record.sh\\\"\" >> ~/.bashrc
    echo \"alias visbot_record_kill=\\\"/home/visbot/ros_ws/src/visbot_scripts/data_record/data_record_kill.sh\\\"\" >> ~/.bashrc
    echo \"alias visbot_rosbag_record=\\\"/home/visbot/ros_ws/src/visbot_scripts/rosbag_record/rosbag_record.sh\\\"\" >> ~/.bashrc
    echo \"alias visbot_rosbag_record_kill=\\\"/home/visbot/ros_ws/src/visbot_scripts/rosbag_record/rosbag_record_kill.sh\\\"\" >> ~/.bashrc
    echo \"alias visbot_raw_record=\\\"/home/visbot/ros_ws/src/visbot_scripts/raw_data_record/vio_raw_data_record.sh\\\"\" >> ~/.bashrc

    find ~/ros_ws/src/visbot_scripts -type f -name \"*.sh\" -exec chmod +x {} \;

    echo -e \"\033[32m~/.bashrc updated and scripts are executable on $ip.\033[0m\"
  }'"

  echo
}

# Main script starts here
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
visbot_scripts_dir="$SCRIPT_DIR/visbot_scripts"
yaml_file="$SCRIPT_DIR/swarm_ip.yaml"
ips=$(read_ips "$selection")

echo "Selected swarm: $selection"
echo "Drone IPs:"
echo "$ips"
echo

for ip in $ips; do
  if ssh -n -f "visbot@$ip" "exit" > /dev/null 2>&1; then
    update_bashrc_and_copy_scripts "$ip"
  else
    echo -e "\033[31mFailed to connect to $ip, no updates made.\033[0m"
    continue
  fi
done

echo -e "\033[1;32mSychronization DONE.\033[0m"
echo