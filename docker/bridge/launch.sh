#!/bin/bash

 ###############################################################################
 # Copyright (c) 2019  Carnegie Mellon University
 #
 # Permission is hereby granted, free of charge, to any person obtaining a copy
 # of this software and associated documentation files (the "Software"), to deal
 # in the Software without restriction, including without limitation the rights
 # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 # copies of the Software, and to permit persons to whom the Software is
 # furnished to do so, subject to the following conditions:
 #
 # The above copyright notice and this permission notice shall be included in
 # all copies or substantial portions of the Software.
 #
 # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 # THE SOFTWARE.
 ###############################################################################

set -e

ulimit -S -c 0
# ulimit -c unlimited
# echo 1 | sudo tee /proc/sys/kernel/core_uses_pid
# echo "/home/developer/core" | sudo tee /proc/sys/kernel/core_pattern
# ulimit -s 65536

function red {
    echo -en "\033[31m"  ## red
    echo $1
    echo -en "\033[0m"  ## reset color
}

if [ "$1" == "build" ]; then
    # unsetting ROS_DISTRO to silence ROS_DISTRO override warning
    unset ROS_DISTRO
    # setup ros1 environment
    source "/opt/ros/$ROS1_DISTRO/setup.bash"

    # unsetting ROS_DISTRO to silence ROS_DISTRO override warning
    unset ROS_DISTRO
    # setup ros2 environment
    source "/opt/overlay_bridge_ws/install/setup.bash"

    colcon build
    if [ $? != 0 ]; then
        red "Error building workscape"
    fi
    exit
else
    echo "Skip building workscape"
fi

exec /home/developer/bridge_ws/launch.sh $@
