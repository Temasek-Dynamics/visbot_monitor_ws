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
echo "1) CustomizedSet"
echo "0) FactoryReset"
read -p "Enter your choice (1 or 0): " choice
echo

if [ "$choice" -eq 1 ]; then
  drone_setting="owl2_customized"
elif [ "$choice" -eq 0 ]; then
  drone_setting="owl2_factory"
else
  echo "Invalid choice. Exiting."
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
drone_dir="$SCRIPT_DIR/owl2/$drone_setting"
drone_yaml="$drone_dir/owl2_config.yaml"
proxy_file="$drone_dir/ros_proxy_drone.launch"

if [ ! -f "$drone_yaml" ]; then
    echo "$drone_yaml not found!"
    exit 1
fi

waypoint_num=$(yq eval '.captain_waypoint_num' $drone_yaml)

for i in $(yq eval '.owl2 | keys' $drone_yaml | grep -o '[0-9]\+'); do
  
  drone_id=$(yq eval ".owl2[$i].Drone_ID" $drone_yaml)

  # Drone parameters
  swarm_id=$(yq eval ".owl2[] | select(.Drone_ID == $drone_id) | .drone_id" $drone_yaml)
  swarm_num=$(yq eval ".owl2[] | select(.Drone_ID == $drone_id) | .drone_num" $drone_yaml)
  drone_ip=$(yq eval ".owl2[] | select(.Drone_ID == $drone_id) | .IP" $drone_yaml)
  tgt_system=$(yq eval ".owl2[] | select(.Drone_ID == $drone_id) | .tgt_system" $drone_yaml)
  MAV_SYS_ID=$(yq eval ".owl2[] | select(.Drone_ID == $drone_id) | .MAV_SYS_ID" $drone_yaml)
  gcs_url=$(yq eval ".owl2[] | select(.Drone_ID == $drone_id) | .gcs_url" $drone_yaml)

  echo "-------------------------------------------------------------------------------"
  echo -e "Updating configuration parameters for OWL 2 Drone $drone_id ($drone_ip)"
  echo "  drone_ip: $drone_ip"
  echo "  swarm_id: $swarm_id"
  echo "  swarm_num: $swarm_num"
  echo "  tgt_system: $tgt_system"
  echo "  MAV_SYS_ID: $MAV_SYS_ID"
  echo "  gcs_url: $gcs_url"
  echo "  waypoint_num: $waypoint_num"

  # 0. Copy ros_proxy_drone.launch to drone
  scp "$proxy_file" visbot@$drone_ip:/home/visbot/ros_ws/src/ros_proxy/launch/ > /dev/null 2>&1
  
  if [ $? -eq 0 ]; then
    echo "Copied $drone_setting/ros_proxy_drone.launch to $drone_ip"
    ssh -q visbot@$drone_ip <<EOF | grep -v 'Welcome to' | grep -v '*' | grep -v '__' | grep -v '\\' | grep -v 'Psuedo-terminal' | sed '/^\s*$/d'
    # 1. Update ~/config/swarm_param.yaml
    swarm_param_file="\$HOME/config/swarm_param.yaml"
    sed -i "s/^drone_id:.*/drone_id: $swarm_id/" "\$swarm_param_file"
    sed -i "s/^drone_num:.*/drone_num: $swarm_num/" "\$swarm_param_file"
    echo "Updated \$swarm_param_file"

    # 2. Update ~/ros_ws/install/share/visbot_itof/launch/visbot_itof.launch
    itof_file="\$HOME/ros_ws/install/share/visbot_itof/launch/visbot_itof.launch"
    sed -i "s|tcp://192\.168\.[0-9]*\.[0-9]*|tcp://$drone_ip|g" "\$itof_file"
    echo "Updated \$itof_file"

    # 3. Update ~/ros_ws/install/share/captain/launch/captain.launch
    captain_file="\$HOME/ros_ws/install/share/captain/launch/captain.launch"
    sed -i "s|<param name=\"waypoint_num\" value=\"[0-9]*\"|<param name=\"waypoint_num\" value=\"$waypoint_num\"|" "\$captain_file"
    sed -i "s|<param name=\"waypoint_num\" value=\"null\"|<param name=\"waypoint_num\" value=\"$waypoint_num\"|" "\$captain_file"
    echo "Updated \$captain_file"

    # 4. Update ~/ros_ws/src/mavros/mavros/launch/px4_visquad.launch
    px4_visquad_file="\$HOME/ros_ws/src/mavros/mavros/launch/px4_visquad.launch"
    
    # #############################
    # For debugging
    # #############################
    # echo "Updating file: \$px4_visquad_file"
    # echo "GCS URL: ${gcs_url}"
    # echo "TGT SYSTEM: ${tgt_system}"
    # echo "Before modification:"
    # grep "gcs_url" "\$px4_visquad_file"
    # grep "tgt_system" "\$px4_visquad_file"
    # #############################

    sed -i "s|<arg name=\"gcs_url\" default=\"[^\"]*\"|<arg name=\"gcs_url\" default=\"$gcs_url\"|" "\$px4_visquad_file"
    sed -i "s|<arg name=\"tgt_system\" default=\"[0-9]*\"|<arg name=\"tgt_system\" default=\"$tgt_system\"|" "\$px4_visquad_file"
    echo "Updated \$px4_visquad_file"

    # #############################
    # For debugging
    # #############################
    # echo "After modification:"
    # grep "gcs_url" "\$px4_visquad_file"
    # grep "tgt_system" "\$px4_visquad_file"
    # #############################

    # 5. Set MAV_SYS_ID
    source /opt/ros/noetic/setup.bash
    mavros_output=\$(rosservice call /mavros/param/set "{param_id: 'MAV_SYS_ID', value: {integer: $MAV_SYS_ID, real: 0.0}}")

    if echo "\$mavros_output" | grep -q "success: True"; then
        echo "Updated MAV_SYS_ID to $MAV_SYS_ID"
    else
        echo -e "\033[31mFailed to update MAV_SYS_ID\033[0m"
    fi  

    echo -e "Please \033[32mManually Reboot $drone_ip\033[0m to apply changes."
EOF
  
  else
    echo "Failed to connect to $drone_ip"
  fi

  echo "-------------------------------------------------------------------------------"
  echo

done