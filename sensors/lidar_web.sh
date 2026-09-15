#!/bin/bash
# start the lidar web GUI detached; log in /tmp/lidar_web.log
cd "$(dirname "$0")"
pgrep -f "python3 lidar_web" >/dev/null && { echo already running; exit 0; }
nohup python3 lidar_web.py --autostart "$@" > /tmp/lidar_web.log 2>&1 &
sleep 1; echo started pid $!; echo "http://$(hostname -I | cut -d" " -f1):8080"
