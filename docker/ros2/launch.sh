#!/bin/bash

# ulimit -c unlimited
# echo 1 | sudo tee /proc/sys/kernel/core_uses_pid
# echo "/home/developer/core" | sudo tee /proc/sys/kernel/core_pattern
# ulimit -s 65536

exec ./script/cabot_ros2.sh $@
