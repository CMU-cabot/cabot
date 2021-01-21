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

trap ctrl_c INT TERM

function ctrl_c() {
    echo "killing all ros nodes..."
    rosnode list | grep record | while read -r line; do rosnode kill $line; done
    snore 3
    rosnode kill -a
    snore 3
    
    for pid in ${pids[@]}; do
	echo "killing $pid..."
	kill -s 15 $pid
    done
    for pid in ${pids[@]}; do
	wait $pid
    done

    gzc=0
    while [ `ps -A | grep gz | wc -l` -ne 0 ];
    do
	snore 1
	echo -ne "waiting gazebo is completely terminated ($gzc)"\\r
	gzc=$((gzc+1))
    done
    echo \\n
    exit
}

function snore()
{
    local IFS
    [[ -n "${_snore_fd:-}" ]] || exec {_snore_fd}<> <(:)
    read ${1:+-t "$1"} -u $_snore_fd || :
}

bagfile=0
start=0
rate=1
info=0

command='xterm -e'
commandpost='&'
pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

while [ ${PWD##*/} != "catkin_ws" ]; do
    cd ..
done
catkin_ws=`pwd`

pids=()

function usage {
    echo "Usage"
    echo "ex)"
    echo $0 "-b xxx.bag"
    echo ""
    echo "-h                       show this help"
    echo "-d                       debug (keep xterm after killing)"
    echo "-b <bagfile>             specify a bagfile"
    echo "-s <seconds>             start time"
    echo "-r <rate>                publish rate"
    echo "-i                       show info"
    exit
}

while getopts "hb:ds:r:i" arg; do
    case $arg in
	h)
	    usage
	    exit
	    ;;

	b)
	    bagfile=$OPTARG
	    ;;
	d)
	    command="xterm -e '"
	    commandpost=";read'&"
	    ;;
	s)
	    start=$OPTARG
	    ;;
	r)
	    rate=$OPTARG
	    ;;
	i)
	    info=1
	    ;;
    esac
done

source $catkin_ws/devel/setup.bash

cd $pwd
cd $(dirname $bagfile)
bagdir=`pwd`
bagname=$(basename $bagfile)
bagfile=$bagdir/$bagname

echo "bagfile is $bagfile"

if [ $info -eq 1 ]; then
    rosbag info $bagfile
    exit
fi

rosnode list
if [ $? -eq 1 ]; then
    eval "$command roscore $commandpost"
    snore 1
fi

echo "launch rviz"
eval "$command roslaunch cabot_ui view_cabot.launch $commandpost"
pids+=($!)

eval "$command roslaunch cabot_description cabot.launch offset:=0.25 $commandpost"

echo "$command rosbag play -s $start -r $rate $bagfile $commandpost"
eval "$command rosbag play -s $start -r $rate $bagfile $commandpost"
pids+=($!)

for pid in ${pids[@]}; do
    wait $pid
done
