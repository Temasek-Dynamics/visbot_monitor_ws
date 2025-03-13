cmake_minimum_required(VERSION 2.8.3)
project(controller_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs sensor_msgs message_generation std_msgs)



add_message_files(
  DIRECTORY msg
  FILES
  FlatTarget.msg
  YawTarget.msg
  control.msg
  cmd.msg
  DroneState.msg
  TargetPose.msg
  PoseStamped.msg
  MinTraj.msg
  Trajectory.msg
  SwarmMatrix.msg
  TargetCompletion.msg
)

generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

catkin_package(
  CATKIN_DEPENDS geometry_msgs sensor_msgs message_runtime std_msgs)
