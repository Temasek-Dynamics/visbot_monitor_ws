#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

printf '%*s\n' "$(tput cols)" '' | tr ' ' '-'
echo -e "\033[1;33mExperiment Data Record\033[0m"
echo

read -p "Manually Specify (Reset) Experiment Number? (y/n): " yn
case $yn in
    [Yy]* ) 
        read -p "Enter Experiment Number: " EXP_NUM;;
    [Nn]* ) EXP_NUM="";;
    * ) 
        echo "Invalid input, please enter 'y' or 'n'." 
        exit 1;;
esac

echo
"$SCRIPT_DIR/data_record/swarm_record.sh" "$EXP_NUM"