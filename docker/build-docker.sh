#!/bin/bash

# Copyright (c) 2020, 2021  Carnegie Mellon University, IBM Corporation, and others
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
    echo "-p                    project name"
    echo "-P                    prebuild"
    echo "-n                    no cache"
}

time_zone=`cat /etc/timezone`
prebuild=0
option="--progress=tty"
debug=0
pwd=`pwd`
prefix=`basename $pwd`

export DOCKER_BUILDKIT=1
export COMPOSE_DOCKER_CLI_BUILD=1

while getopts "ht:p:Pndc:" arg; do
    case $arg in
	h)
	    help
	    exit
	    ;;
	c)
	    CUDAV=$OPTARG
	    ;;
	t)
	    time_zone=$OPTARG
	    ;;
	p)
	    prefix=$OPTARG
	    ;;
	P)
	    prebuild=1
	    ;;
	n)
	    option="$option --no-cache"
	    ;;
	d)
	    debug=1
	    ;;
    esac
done
shift $((OPTIND-1))
target=$1
if [ "$target" = "" ]; then
    target=all
fi

image_l=${prefix}_nvidia-cuda11.1-cudnn8-devel-glvnd-runtime-ros-base-ubuntu20.04
image_p=${prefix}_nvidia-cuda11.1-cudnn8-devel-glvnd-runtime-ros-base-realsense-ubuntu20.04
if [ $target = "people" ] || [ $target = "all" ]; then
    if [ `docker images | grep $image_p | wc -l` = 0 ]; then
	red "You are trying to build with CUDA$CUDAV, but cannot find the corresponding images"
	echo " - $image_p"
	echo ""
	red "You need to run ./prebuild.sh -c <CUDA version string like '10.1'> first"
	echo " Please check your CUDA version"
	blue "`nvidia-smi | grep CUDA`"
	echo ""
	help
	exit
    fi	
fi


if [ "$target" = "ros1" ] || [ "$target" = "all" ]; then
    cmd="docker-compose -p ${prefix} build $option --build-arg UID=$UID --build-arg TZ=$time_zone ros1"
    blue $cmd
    eval $cmd
    if [ $? != 0 ]; then
	red "Got an error to build ros1"
	exit
    fi
    docker-compose -p ${prefix} run ros1 catkin_make
    if [ $? != 0 ]; then
	red "Got an error to build ros1 ws"
	exit
    fi
fi


if [ "$target" = "bridge" ] || [ "$target" = "all" ]; then
    cmd="docker-compose -p ${prefix} build $option --build-arg UID=$UID --build-arg TZ=$time_zone bridge"
    blue $cmd
    eval $cmd
    if [ $? != 0 ]; then
	red "Got an error to build bridge"
	exit
    fi
    docker-compose -p ${prefix} run bridge ./launch.sh build
    if [ $? != 0 ]; then
	red "Got an error to build bridge ws"
	exit
    fi
fi


if [ "$target" = "ros2" ] || [ "$target" = "all" ]; then
    if [ $prebuild -eq 1 ]; then
	./prebuild.sh -t $time_zone
    fi
    
    image_n=galactic-ros-desktop-nav2-focal
    if [ `docker images | grep $image_n | wc -l` = 0 ]; then
	red "cannot find the corresponding images"
	echo " - $image_n"
	red "You need to run ./prebuild.sh first or add -p option"
	echo ""
	help
	exit
    fi

    cmd="docker-compose -p ${prefix} build $option --build-arg UID=$UID --build-arg TZ=$time_zone ros2"
    blue $cmd
    eval $cmd
    if [ $? != 0 ]; then
	red "Got an error to build ros2"
	exit
    fi
    if [ $debug -eq 1 ]; then
	docker-compose -p ${prefix} run ros2 colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
    else
	docker-compose -p ${prefix} run ros2 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
    fi

    if [ $? != 0 ]; then
	red "Got an error to build ros2 ws"
	exit
    fi
fi


if [ $target = "localization" ] || [ $target = "all" ]; then
    docker-compose -p ${prefix} build \
		   --build-arg FROM_IMAGE=$image_l \
		   --build-arg UID=$UID \
		   --build-arg TZ=$time_zone \
		   $option \
		   localization
    if [ $? != 0 ]; then
	red "Got an error to build localization"
	exit
    fi
    cmd="docker-compose -p ${prefix} -f docker-compose-mapping.yaml build  \
		   --build-arg FROM_IMAGE=$image_l \
		   --build-arg UID=$UID \
		   --build-arg TZ=$time_zone \
		   $option \
		   topic_checker"
    echo $cmd
    eval $cmd
    if [ $? != 0 ]; then
	red "Got an error to build topic_checker"
	exit
    fi
    docker-compose -p ${prefix} run localization /launch.sh build
    if [ $? != 0 ]; then
	red "Got an error to build localization ws"
	exit
    fi
    docker-compose -p ${prefix} -f docker-compose-mapping.yaml run localization /launch.sh build
    if [ $? != 0 ]; then
	red "Got an error to build localization ws"
	exit
    fi
fi


if [ $target = "people" ] || [ $target = "all" ]; then
    docker-compose -p ${prefix} build \
		   --build-arg FROM_IMAGE=$image_p \
		   --build-arg UID=$UID \
		   --build-arg TZ=$time_zone \
		   $option \
		   people
    if [ $? != 0 ]; then
	red "Got an error to build people"
	exit
    fi
    docker-compose -p ${prefix} run people /launch.sh build
    if [ $? != 0 ]; then
	red "Got an error to build people ws"
	exit
    fi
fi


if [ $target = "wireless" ] || [ $target = "all" ]; then
    wifi_int=`iw dev | grep Interface | cut -f2 -d' '`
    if [ -e .env ]; then
	sed "s/WIFI_INTERFACE=.*/WIFI_INTERFACE=$wifi_int/g" -i .env
    else
	echo "WIFI_INTERFACE=$wifi_int" > .env
    fi
    blue "Wifi Interface: $wifi_int"

    docker-compose -p ${prefix} -f docker-compose-mapping.yaml build  \
		   --build-arg TZ=$time_zone \
		   $option \
		   wifi_scan
    if [ $? != 0 ]; then
	red "Got an error to build wifi_scan"
	exit
    fi

    docker-compose -p ${prefix} -f docker-compose-mapping.yaml build  \
		   --build-arg TZ=$time_zone \
		   $option \
		   ble_scan
    if [ $? != 0 ]; then
	red "Got an error to build ble_scan"
	exit
    fi
fi

