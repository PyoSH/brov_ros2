#!/usr/bin/env bash
cd ~/BROV/brov_ros2 && source env_native.sh
ros2 service call /brov/stop_control   std_srvs/srv/Trigger
ros2 service call /brov/disarm_control std_srvs/srv/Trigger
