#!/bin/bash

#CMDDIR="$HOME/bin/cmd"
CMDDIR="$PWD/cmd"
MSG="/control"
MSGTYPE="controller_msgs/control"


if test $# != 1; then
   echo "Usage: $0 [takeoff|landing|start|stoptask|shape|track|hit|follow|swmatrix]"
   exit
fi

case $1 in 
    "takeoff")
        cmd=takeoff
        ;;
    "landing")
        cmd=landing
        ;;
    "start")
        cmd=start
        ;;
    "stoptask")
        cmd=stoptask
        ;;
    "setpoint")
        cmd=setpoint
        ;;
    "track")
        cmd=track
        ;;
    "hit")
        cmd=hit
        ;;
    "follow")
        cmd=follow
        ;;
    "swmatrix")
        MSG="/swarm_matrix"
        MSGTYPE="controller_msgs/SwarmMatrix"
        cmd=swmatrix
        ;;
    *)
        echo "wrong cmd:$1"
        exit
        ;;
esac

if [[ ! -f "$CMDDIR/$cmd.msg" ]]; then
    echo "$CMDDIR/$cmd.msg does not exist!"
    exit
fi

echo cmd:$cmd 
#echo rostopic pub -1 /control controller_msgs/control -f $CMDDIR/$cmd.msg
rostopic pub -1 $MSG $MSGTYPE -f $CMDDIR/$cmd.msg

