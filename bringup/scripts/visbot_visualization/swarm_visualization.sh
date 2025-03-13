#!/bin/bash

#####
# Directories
#####
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )/.."
echo "$SCRIPT_DIR"


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
roslaunch ros_proxy ros_proxy_commander.launch 
"

#####
# Execute
#####
$SOURCE_WS
$CMD_0