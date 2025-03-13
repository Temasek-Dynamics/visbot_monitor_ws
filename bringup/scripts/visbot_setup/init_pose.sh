#!/bin/bash

SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )/.."
source /opt/ros/noetic/setup.bash
source $SCRIPT_DIR/../../devel/setup.bash

ros_package="visbot_monitor"
config_path=$(rospack find $ros_package)/config/init_pose

if [ ! -d "$config_path" ]; then
  echo "Error: config folder not found in the package $ros_package"
  exit 1
fi

yaml_files=($(find "$config_path" -maxdepth 1 -name "*.yaml" | sort))
yaml_names=()
for file in "${yaml_files[@]}"; do
  yaml_names+=($(basename "$file" .yaml))
done

if [ ${#yaml_files[@]} -eq 0 ]; then
  echo "No YAML files found in $config_path!"
  exit 1
fi

PS3="Enter Initial Configuration Number (1, 2, 3, ...): "
echo "Please Select Initial Pose:"
select init_file in "${yaml_names[@]}"; do
  echo
  if [ -n "$init_file" ]; then
    echo "Selected Initial Configuration: $init_file"
    break
  else
    echo "Invalid selection. Please try again."
  fi
done

echo
echo -e "\033[36mROS Output\033[0m"
printf '%*s\n' "$(tput cols)" '' | tr ' ' '-'

roslaunch $ros_package init_pose.launch yaml_file:="$config_path/$init_file.yaml" 2>/dev/null