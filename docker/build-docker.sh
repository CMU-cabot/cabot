#!/bin/bash

# Copyright (c) 2020 Carnegie Mellon University
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


function red {
    echo -en "\033[31m"  ## red
    echo $1
    echo -en "\033[0m"  ## reset color
}
function blue {
    echo -en "\033[36m"  ## blue
    echo $1
    echo -en "\033[0m"  ## reset color
}
function help {
    echo "Usage: $0 <option>"
    echo ""
    echo "-h                    show this help"
    echo "-t <time_zone>        set time zone"
    echo "-p                    prebuild"
    echo "-n                    no cache"
}

time_zone=`cat /etc/timezone`
prebuild=0
option=

while getopts "ht:pn" arg; do
    case $arg in
	h)
	    help
	    exit
	    ;;
	t)
	    time_zone=$OPTARG
	    ;;
	p)
	    prebuild=1
	    ;;
	n)
	    option="$option --no-cache"
	    ;;
    esac
done
shift $((OPTIND-1))

blue "TIME_ZONE=$time_zone"

target=$1

if [ "$target" = "" ]; then
    target=all
fi

if [ "$target" = "ros1" ] || [ "$target" = "all" ]; then
    cmd="docker-compose build $option --build-arg UID=$UID --build-arg TZ=$time_zone ros1"
    blue $cmd
    eval $cmd
    if [ $? != 0 ]; then
	red "Got an error to build ros1"
	exit
    fi
fi


if [ "$target" = "bridge" ] || [ "$target" = "all" ]; then
    cmd="docker-compose build $option --build-arg UID=$UID --build-arg TZ=$time_zone bridge"
    blue $cmd
    eval $cmd
    if [ $? != 0 ]; then
	red "Got an error to build bridge"
	exit
    fi
fi


if [ "$target" = "ros2" ] || [ "$target" = "all" ]; then
    if [ $prebuild -eq 1 ]; then
	./prebuild.sh -t $time_zone
    fi
    
    image_n=foxy-ros-desktop-nav2-focal
    if [ `docker images | grep $image_n | wc -l` = 0 ]; then
	red "cannot find the corresponding images"
	echo " - $image_n"
	red "You need to run ./prebuild.sh first or add -p option"
	echo ""
	help
	exit
    fi

    cmd="docker-compose build $option --build-arg UID=$UID --build-arg TZ=$time_zone ros2"
    blue $cmd
    eval $cmd
    if [ $? != 0 ]; then
	red "Got an error to build ros2"
	exit
    fi
fi


if [ $target = "ws" ] || [ $target = "all" ]; then
    blue "bulid ros1 ws"
    docker-compose run ros1 catkin_make
    if [ $? -ne 0 ]; then red "failed to build cabot_ros"; fi
    
    blue "bulid bridge ws"
    docker-compose run bridge ./launch.sh build
    if [ $? -ne 0 ]; then red "failed to build bridge"; fi
    
    blue "bulid ros2 ws"
    docker-compose run ros2 colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo 
    if [ $? -ne 0 ]; then red "failed to build cabot_ros2"; fi
fi

