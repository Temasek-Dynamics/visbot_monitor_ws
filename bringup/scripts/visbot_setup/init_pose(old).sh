#!/bin/bash


#####
# Directories
#####
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )/.."

#####
# Sourcing
#####
SOURCE_WS="
source $SCRIPT_DIR/../../devel/setup.bash 
"

#####
# Commands
#####
# Start drones with planner modules
CMD_0="
roslaunch visbot_monitor init_pose.launch
"

#####
# Execute
#####
$SOURCE_WS
$CMD_0