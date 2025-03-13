#!/bin/bash

if [ -z "$1" ]; then
  echo "Usage: $0 <FOLDER_PATH>"
  exit 1
fi

FOLDER_PATH=$1

LOGDIR="$HOME/log"
LOGCUR=$(ls -d "$LOGDIR"/log* 2>/dev/null | sort -V | tail -n 1)
LOG_PATH="$FOLDER_PATH/logs"

mkdir -p $LOG_PATH

setsid bash -c "
    tail -f $LOGCUR/visbot_ntp.log | tee $LOG_PATH/visbot_ntp.log > /dev/null &
    tail -f $LOGCUR/sys_monitor.log | tee $LOG_PATH/sys_monitor.log > /dev/null &
    tail -f $LOGCUR/stereo_cam.log | tee $LOG_PATH/stereo_cam.log > /dev/null &
    tail -f $LOGCUR/mavros.log | tee $LOG_PATH/mav_ros.log > /dev/null &
    tail -f $LOGCUR/vins.log | tee $LOG_PATH/vins.log > /dev/null &
    tail -f $LOGCUR/planner.log | tee $LOG_PATH/planner.log > /dev/null &
    tail -f $LOGCUR/mavros_controller.log | tee $LOG_PATH/mavros_controller.log > /dev/null &
    tail -f $LOGCUR/captain.log | tee $LOG_PATH/captain.log > /dev/null &
    tail -f $LOGCUR/proxy.log | tee $LOG_PATH/proxy.log > /dev/null &
    tail -f $LOGCUR/gimbal.log | tee $LOG_PATH/gimbal.log > /dev/null &
" & # tail -f $LOGCUR/visbot_media.log | tee $LOG_PATH/visbot_media.log > /dev/null &

echo $! > /tmp/redirect_log_pgid.txt
echo "Logs are redirected to $LOG_PATH"