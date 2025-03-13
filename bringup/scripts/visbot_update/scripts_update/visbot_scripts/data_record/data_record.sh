#!/bin/bash

DATE=$(date +"%Y-%m-%d")
TIME_STAMP=$(date +"%Y-%m-%d_%H-%M-%S")
DRONE_ID=`cat /home/visbot/config/swarm_param.yaml | grep drone_id | awk -F " "  '{print $2}'`

FOLDER_PATH="$HOME/UserData/experiments/$DATE"
mkdir -p "$FOLDER_PATH"

EXP_NUM=$1

if [ -z "$EXP_NUM" ]; then
    EXP_DIRS=$(ls -d "$FOLDER_PATH"/exp* 2>/dev/null | grep -oP 'exp\K\d+')
    
    if [ -z "$EXP_DIRS" ]; then
        EXP_NUM=1
    else
        MAX_EXP_NUM=$(echo "$EXP_DIRS" | sort -n | tail -n 1)
        EXP_NUM=$((MAX_EXP_NUM + 1))
    fi
# #######################################################
# No tests made, and may lead to unexpected error if the 
# lastest recordings are not be terminated properly
# #######################################################
# else
#     if [ -d "$FOLDER_PATH/exp${EXP_NUM}" ]; then
#         SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
#         "${SCRIPT_DIR}/data_record_kill.sh > /dev/null 2>&1"
        
#         SUFFIX=1
#         while [ -d "$FOLDER_PATH/exp${EXP_NUM}_deprecated_$SUFFIX" ]; do
#             SUFFIX=$((SUFFIX + 1))
#         done

#         mv "$FOLDER_PATH/exp${EXP_NUM}" "$FOLDER_PATH/exp${EXP_NUM}_deprecated_$SUFFIX"
        
#         echo "Exp${EXP_NUM} exists. Renamed the directory to exp${EXP_NUM}_deprecated_$SUFFIX..."
#     fi
fi

FOLDER_PATH="$FOLDER_PATH/exp${EXP_NUM}"
mkdir -p "$FOLDER_PATH"

printf '%*s\n' "${COLUMNS:-80}" '' | tr ' ' '#'
echo "$TIME_STAMP, Drone $DRONE_ID, Experiment $EXP_NUM"

# Echo the latest log
LOGDIR="$HOME/log"
LATESTLOG=$(ls -d "$LOGDIR"/log* 2>/dev/null | sort -V | tail -n 1)

if [ -z "$LATESTLOG" ]; then
    echo "No log folders found."
else
    LOGNUM=$(basename "$LATESTLOG" | grep -oP '\d+')
    echo "The latest log number for Drone $DRONE_ID is: $LOGNUM"
fi

# Find the script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Redirect the log
"${SCRIPT_DIR}/redirect_log.sh" "$FOLDER_PATH"

# Record the rosbag
"${SCRIPT_DIR}/record_rosbag.sh" "$FOLDER_PATH"

printf '%*s\n' "${COLUMNS:-80}" '' | tr ' ' '#'