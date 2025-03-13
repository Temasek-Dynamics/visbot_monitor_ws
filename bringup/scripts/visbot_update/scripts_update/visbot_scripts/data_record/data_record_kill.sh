#!/bin/bash

# Find the script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

"${SCRIPT_DIR}/redirect_log_kill.sh"
"${SCRIPT_DIR}/record_rosbag_kill.sh"