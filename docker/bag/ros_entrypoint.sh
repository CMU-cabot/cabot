#!/bin/bash
set -e

# setup ros2 environment
source "/home/developer/bag_ws/install/setup.bash" --
exec "$@"
