# Visbot Monitor

## Description
This README file is outdated. Please refer to the latest version of the README in `visbot_monitor_ws/src/visbot_monitor/README.md`.
This package is developed to visualize obstacles, trajectories, and odometry of visbot swarm, also can publish target pose and task to the swarm.

## 1. Compilation
To build the project, use the following command:
```shell
catkin build
```
# A. Latest version:
```shell
cd visbot_monitor_ws/bringup/scripts
./monitor.sh
```
## 2. Launching the Ground Station
```shell
cd visbot_monitor_ws/bringup/scripts
./monitor.sh
```
## 3. Predefine the swarm Init pose
1. Setting the file:
```shell
visbot_monitor_ws/src/visbot_monitor/visbot_monitor/launch/init_pose.launch
```
2. Run the  command:
```shell
cd visbot_monitor_ws/bringup/scripts
./init_pose_send.sh
```

## 4. Publish the swarm matrix and the swarm target position
1. Setting both the swarm matrix and the swarm target position in the file:
```shell
visbot_monitor_ws/src/visbot_monitor/visbot_monitor/launch/sequential_waypoint.launch
```
2. Run the command (***After the drone has taken off***):
```shell
cd visbot_monitor_ws/bringup/scripts
./mission.sh
```
Note: the nameing of the mission.sh is because the mission may include other tasks except the sequential waypoint in the future.

# Below is the original version:

## 2. Launching the Ground Station
To launch the ground station, execute:
```shell
roslaunch ros_proxy ros_proxy_commander.launch
```

## 3. Publishing Targets
To correctly publish targets with drone_id, run:
```shell
rosrun visbot_monitor goal_repub.py
```

## 4. Managing swmatrix

### Publishing swmatrix
To publish swmatrix, follow these steps:
```shell
cd src/visbot_monitor/cmd
chmod +x cmd.sh
./cmd.sh swmatrix
```

### Modifying swmatrix
To modify swmatrix, you can edit the `swmatrix.msg` file:
```shell
cd src/visbot_monitor/cmd/cmd
vim swmatrix.msg
```
