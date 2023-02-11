#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/install/setup.bash"

exec "$@"
