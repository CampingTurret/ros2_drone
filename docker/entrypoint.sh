#!/bin/bash

# Navigate to the right directory and source it
cd /home/user/imav_ws
. /opt/ros/${ROS_DISTRO}/setup.sh
. install/setup.bash

exec "$@"