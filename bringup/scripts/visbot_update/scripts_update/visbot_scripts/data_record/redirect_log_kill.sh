#!/bin/bash

PGID=$(cat /tmp/redirect_log_pgid.txt)

if [ -n "$PGID" ]; then
    echo "Killing process group $PGID"
    kill -TERM -$PGID
    rm -rf /tmp/redirect_logs_pgid.txt
else
    echo "No process group ID found."
fi