# `visbot_update` Scripts Descriptions

This directory contains scripts for updating the drones and the ground control station (GCS).

## Example: OWL3 WiFi Switch and Parameters Update

Here is an example of how to switch the owl3 swarm WiFi configuration from `visbot_ap` to `visbot_net_5GHz` and update drones' parameters:

0. Connect the drones and the GCS to the `visbot_ap` network.
1. Run `ssh_connect/ssh_connect.sh` on GCS and select `owl3_factory`.
2. Run `wifi_sync/owl3/wifi_switch.sh` and select `ap2net`.
3. Manually switch the GCS Wi-Fi to the network set by `wifi_switch.sh`, i.e., `visbot_net_5GHz`.
4. Run `ssh_connect/ssh_connect.sh` and select `owl3`.
5. Run `config_update/owl3_update.sh` and choose `CustomizedSet`.
6. Manually reboot the drone to complete the configuration.


## ssh_connect.sh
`ssh_connect.sh`: Connect the drones and the GCS to the same WiFi network first. After establishing the connection, run `ssh_connect.sh` and select the corresponding drone swarm to update the ground station's SSH key. Available options include:
- **owl2** (192.168.2.20-23): The GCS connects to the owl2 swarm.
- **owl3** (192.168.2.30-37): The GCS connects to the owl3 swarm.
- **owl2 & owl3**: The GCS connects to both owl2 and owl3 swarms.
- **owl3_factory** (192.168.2.20-27): If the owl2 swarm was previously connected to the GCS via SSH, the owl3 swarm will replace the owl2 swarm registration.

## scripts_sync.sh
This script updates the `visbot_scripts` for data recording. First, connect the drones and the ground station to the same network. Then, run `scripts_sync.sh` and choose the drone swarm to update:

- **owl2**
- **owl3**
- **owl2 & owl3**
- **owl3_factory**

After doing so, the scripts located at `~/ros_ws/src/visbot_scripts` within the drones' onboard computers will be updated accordingly.

## wifi_switch.sh
This script switches the WiFi configuration for the drones by updating `/config/etc/wifi_config.sh`. Connect the drones and the ground station to the same network first, then run `wifi_switch.sh`. You can choose:

- **ap2net**: Modify drones' IP address from 192.168.2.2* -> 192.168.2.3*
- **net2ap**: Modify drones' IP address from 192.168.2.3* -> 192.168.2.2*

**IMPROTANT NOTE:** This script is only compatible with owl3 drones. Do **NOT** use it with owl2 drones, especially the `ap2net` option.

## wifi_ping.sh
This script tests whether the drones and the ground station are successfully connected. After running `wifi_ping.sh`, select the router (either `visbot_ap` or `visbot_net_5GHz`) to check the connection.

## owl3_update.sh
`owl3_update.sh` updates configuration parameters for the `owl3` drones by following the steps below:

1. Before running the script, ensure that the drones and the ground station are connected to the same network, either by running `wifi_switch.sh` or setting the GCS WiFi manually.

    - **For FactoryReset**, ensure all third-generation drones have IPs in the 20 range.
    - **For CustomizedSet**, ensure all third-generation drones have IPs in the 30 range.

2. Wait for all drone ROS nodes to start, and verify that the GCS can communicate with PX4 via MAVROS. If communication between the drone onboard computer and PX4 is disrupted, the script will not work properly.

    **NOTE**: Ensure that the `tgt_system` and `MAV_SYS_ID` are consistent before and after running `owl3_update.sh`.

    **NOTE**: The WiFi configuration does not affect communication between the drone onboard computer and PX4.


3. Run `owl3_update.sh` to update all drone parameters. 
- **CustomizedSet** can only be used when drone IPs are in the `192.168.2.3*` range.
- **FactoryReset** can only be used when drone IPs are in the `192.168.2.2*` range.
