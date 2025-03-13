This deprecated directory includes the scripts that were used to record rosbag from the robots and transfer the recorded rosbag to the gcs.
The scripts in this directory can only be used with the OWL2 swarm and are included here for the reference purpose.

The scripts in this directory are relied on the designed swarm_id.yaml, which is not satisfied with the OWL3 swarm configuration.
Now the gcs uses swarm_ip.yaml to configure the swarm.

The recording scripts on onboard computers are now relocated in ~/ros_ws/src/visbot_scripts, instead of ~/UserData/scripts/rosbag_record.
Hence swarm_record.sh in this directory are no longer supported.

alias 'rosbag_record' still works for onboard computer.
