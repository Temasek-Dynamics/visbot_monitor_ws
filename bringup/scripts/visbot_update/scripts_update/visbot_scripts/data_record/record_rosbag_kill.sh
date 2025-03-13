#!/bin/bash

source /opt/ros/noetic/setup.bash

rosnode kill /rosbag_record_node
rm -rf ./record_rosbag.log 