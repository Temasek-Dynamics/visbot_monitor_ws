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
yaml_file="$SCRIPT_DIR/swarm_ip.yaml"
reference_ip="192.168.2.30"
reference_hostname=$(ssh visbot@$reference_ip 'hostname')
gcs_ip=$(hostname -I | awk '{print $1}')
time_tolerance=1000 # unit: ms (millisecond)
ips=$(read_ips "$selection")

echo -e "Selected Swarm: \033[32m$selection\033[0m"
echo
echo "GCS IP: $gcs_ip"
echo
echo "Drone IPs:"
echo "$ips"
echo

read -p "Synchronize swarm time with GCS? (y/n):" yn
case $yn in
    [Yy]* ) sync_choice="0";;
    [Nn]* ) sync_choice="1";;
    * ) 
        echo "Invalid input, please enter 'y' or 'n'." 
        exit 1;;
esac

echo

successful_ips=()
is_all_connected=true
if [ "$sync_choice" = "0" ]; then
    echo "Enter GCS password to restart 'systemd-timesyncd' service:"
    sudo -k
    sudo systemctl restart systemd-timesyncd
    echo

    # Sync swarm time with GCS
    echo -e "\033[36mSynchronizing swarm time with GCS...\033[0m"
    for ip in $ips; do
        if ssh -n -f "visbot@$ip" "exit" > /dev/null 2>&1; then
            ssh visbot@"$ip" "sudo systemctl disable ntp.service" > /dev/null 2>&1
            local_time=$(date +%m%d%H%M%Y.%S)
            # ssh visbot@"$ip" "sudo date $local_time > /dev/null 2>&1 && nohup sudo /usr/sbin/ntpdate -b -u $gcs_ip > /dev/null 2>&1 &"
            ssh visbot@"$ip" "nohup bash -c 'sudo date $local_time > /dev/null 2>&1 && sudo /usr/sbin/ntpdate -b -u $gcs_ip > /dev/null 2>&1' &"
            successful_ips+=("$ip")
            echo -e "\033[32m$ip: Time Synchronized\033[0m"
        else
            echo -e "\033[31m$ip\033[0m: SSH connection failed"
            is_all_connected=false
        fi
    done
    
    ips="${successful_ips[@]}"

    sleep 5
    
    echo
    echo -e "Local (GCS) time: \033[32m$(date +"%Y-%m-%d %H:%M:%S.%3N")\033[0m"
    for ip in $ips; do
        echo -e "Remote time from $ip: \033[32m$(ssh visbot@"$ip" 'date +"%Y-%m-%d %H:%M:%S.%3N"')\033[0m"
        ((i++))
    done
else
    echo "Time sync with GCS skipped."
fi

echo

# Write the synchronized time to hardware
for ip in $ips; do
    if ssh -n -f "visbot@$ip" "exit" > /dev/null 2>&1; then
        ssh visbot@"$ip" "sudo hwclock --systohc"
    fi
done

# Find time differences between drones and local (GCS)
time_diffs_local=()
time_diff_info_local=()
remote_time=0

if [ "$sync_choice" = "0" ]; then
    for ip in $ips; do
        if ssh -n -f "visbot@$ip" "exit" > /dev/null 2>&1; then
            # Visbot drones is UTC, Local GCS is UTC +8 
            timezone_offset=$(date +%:z)

            offset_hours=$(echo "${timezone_offset:1:2}" | bc)
            offset_sign=${timezone_offset:0:1}

            if [ "$offset_hours" -eq 0 ]; then
                offset_ms=0 # UTC
            else
                if [ "$offset_sign" = "+" ]; then
                    offset_ms=$((-offset_hours * 3600 * 1000))
                else
                    offset_ms=$((offset_hours * 3600 * 1000))
                fi
            fi

            # echo "Timezone offset: $timezone_offset"
            # echo "Offset in milliseconds: $offset_ms"

            local_time_reference=$(( $(date +%s%3N) - offset_ms ))
            remote_time=$(ssh visbot@"$ip" 'date +%s%3N')
            
            time_diff_local=$((remote_time - local_time_reference))
            time_diffs_local+=("$time_diff_local")
            time_diff_info_local+=("$ip: $time_diff_local ms")
            
            echo -e "\033[32m$ip\033[0m: Time Difference with GSC: $time_diff_local ms"
        else
            echo -e "\033[31m$ip\033[0m: SSH connection failed"
        fi
    done
    echo
fi

# Find time differences between drones and reference drone (Drone 0)
time_diffs_remote=()
time_diff_info_remote=()
remote_time=0

for ip in $ips; do
    if ssh -n -f "visbot@$ip" "exit" > /dev/null 2>&1; then
        remote_time_reference=$(ssh visbot@"$reference_ip" 'date +%s%3N')
        remote_time=$(ssh visbot@"$ip" 'date +%s%3N')
        
        # echo "Reference time from $reference_ip: $(ssh visbot@"$reference_ip" 'date +"%Y-%m-%d %H:%M:%S"') (UNIX Timestamp: $reference_time)"
        # echo "Time from $ip: $(ssh visbot@"$ip" 'date +"%Y-%m-%d %H:%M:%S"') (UNIX Timestamp: $remote_time)"

        time_diff_remote=$((remote_time - remote_time_reference))
        time_diffs_remote+=("$time_diff_remote")
        time_diff_info_remote+=("$ip: $time_diff_remote ms")
        
        echo -e "\033[32m$ip\033[0m: Time Difference with $reference_ip: $time_diff_remote ms"
    else
        echo -e "\033[31m$ip\033[0m: SSH connection failed"
    fi
done

# Check time sync with Local (GCS)
if [ "$sync_choice" = "0" ]; then
    all_sync_local=true
    for diff in "${time_diffs_local[@]}"; do
        if (( diff > time_tolerance || diff < -time_tolerance )); then
            all_sync_local=false
            break
        fi
    done

    echo
    if [ "$all_sync_local" = true ]; then
        echo -e "All drones are time synchronized with Local (GCS) within \033[32m$time_tolerance ms.\033[0m"
    else
        echo -e "\033[31mTime Differences with Local (GCS) Detected:\033[0m"
        for info in "${time_diff_info_local[@]}"; do
            echo "$info"
        done
    fi
fi

echo

# Check time sync with reference drone (Drone 0)
all_sync_remote=true
for diff in "${time_diffs_remote[@]}"; do
    if (( diff > time_tolerance || diff < -time_tolerance )); then
        all_sync_remote=false
        break
    fi
done

if [ "$all_sync_remote" = true ]; then
    echo -e "All drones are time synchronized with $reference_hostname ($reference_ip) within \033[32m$time_tolerance ms\033[0m."
else
    echo -e "\033[31mTime Differences with reference $reference_hostname ($reference_ip) Detected:\033[0m"
    for info in "${time_diff_info_remote[@]}"; do
        echo "$info"
    done
fi

echo
if [ "$sync_choice" = "0" ]; then
    if [ "$all_sync_local" = true ] && [ "$all_sync_remote" = true ]; then
        echo -e "\033[1;32mTime Synchronization DONE\033[0m"
    else
        echo -e "\033[31mTime Synchronization NOT COMPLETE\033[0m"
    fi
else
    if [ "$all_sync_remote" = true ]; then
        echo -e "\033[1;32mTime Synchronization DONE\033[0m"
    else
        echo -e "\033[31mTime Synchronization NOT COMPLETE\033[0m"
    fi
fi