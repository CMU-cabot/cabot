#!/bin/bash

# Copyright (c) 2020  Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

while [ ${PWD##*/} != "ros2_ws" ]; do
    cd ..
done
ros2_ws=`pwd`

: ${CABOT_GAZEBO:=0}
: ${CABOT_SITE:=}
: ${CABOT_SHOW_ROS2_RVIZ:=0}
: ${CABOT_SHOW_ROS2_LOCAL_RVIZ:=0}
: ${CABOT_RECORD_ROSBAG2:=0}

amcl=1
pid=
use_cache=0
use_sim_time=false
if [ $CABOT_GAZEBO -eq 1 ]; then use_sim_time=true; fi

# configuration for cabot site scripts
gazebo=$CABOT_GAZEBO   # read by config script

trap ctrl_c INT QUIT TERM

function ctrl_c() {
    echo "killing all process"
    kill -s 2 0
    snore 3
    exit
}

function snore()
{
    local IFS
    [[ -n "${_snore_fd:-}" ]] || exec {_snore_fd}<> <(:)
    read ${1:+-t "$1"} -u $_snore_fd || :
}

function usage {
    echo "Usage"
    echo ""
    echo "-h       show this help"
    echo "-M       turn off amcl"
    echo "-c       use built cache"
    exit
}


while getopts "hvMc" arg; do
    case $arg in
	h)
	    usage
	    exit
	    ;;
	M)
	    amcl=0
	    ;;
	c)
	    use_cache=1
	    ;;
    esac
done
shift $((OPTIND-1))

cd $ros2_ws
if [ $use_cache -eq 0 ]; then
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo 
    if [ $? -ne 0 ]; then
	exit
    fi
else
    echo "Skip building workspace"
fi
source $ros2_ws/install/setup.bash

if [ ! -z $CABOT_SITE ]; then
    sitedir=`ros2 pkg prefix $CABOT_SITE`/share/$CABOT_SITE
    echo $sitedir
    source $sitedir/config/config.sh
    if [ "$map" == "" ]; then
	echo "Please check config/config.sh in site package ($sitedir) to set map and world"
	exit
    fi
else
    if [ "$map" == "" ]; then
	echo "-T <site> or -m <map> should be specified"
	exit
    fi
fi

echo "CABOT_GAZEBO              : $CABOT_GAZEBO"
echo "CABOT_SITE                : $CABOT_SITE"
echo "CABOT_SHOW_ROS2_RVIZ      : $CABOT_SHOW_ROS2_RVIZ"
echo "CABOT_SHOW_ROS2_LOCAL_RVIZ: $CABOT_SHOW_ROS2_LOCAL_RVIZ"
echo "CABOT_RECORD_ROSBAG2      : $CABOT_RECORD_ROSBAG2"
echo "Map                       : $map"
echo "Use AMCL                  : $amcl"
echo "Use Sim Time              : $use_sim_time"

ros2 launch cabot_navigation2 bringup_launch.py \
     map:=$map \
     use_amcl:=$amcl \
     autostart:=true \
     use_sim_time:=$use_sim_time \
     show_rviz:=$CABOT_SHOW_ROS2_RVIZ \
     show_local_rviz:=$CABOT_SHOW_ROS2_LOCAL_RVIZ \
     record_bt_log:=$CABOT_RECORD_ROSBAG2 &

while [ 1 -eq 1 ];
do
    snore 1
done


