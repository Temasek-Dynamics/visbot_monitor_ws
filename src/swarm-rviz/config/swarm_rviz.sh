#!/bin/bash

ROS_MASTER=`cat swarm.cfg | grep ROS_MASTER | awk -F " " '{print $2}'`
export ROS_MASTER_URI="http://$ROS_MASTER:11311"
echo "ROS_MASTER_URI $ROS_MASTER_URI"

#source /home/khadas/ros_ws/devel/setup.bash
hostip=`hostname -I | awk -F " "  '{print $1}'`
if [[ -z $hostip ]]; then
hostip=192.168.2.20
fi
#echo hostip:$hostip
export ROS_IP=$hostip
echo "ROS_IP $ROS_IP"

roslaunch swarm_rviz swarm_rviz.launch 
