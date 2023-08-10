#!/bin/bash

# Copyright (c) 2020, 2023  Carnegie Mellon University, IBM Corporation, and others
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

## termination hook
trap ctrl_c INT QUIT TERM

function ctrl_c() {
    exit
}

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
    echo "$0 [<option>] [<target>]"
    echo ""
    echo "targets : all: all targets"
    echo "          ros2        : build ROS2"
    echo "          localization: build localization"
    echo "          people      : build people"
    echo "          people-nuc  : build people without CUDA"
    echo "          l4t         : build people for jetson"
    echo "          wireless    : build wireless"
    echo "          server      : build server"
    echo "          gnss        : build gnss for outside localization"
    echo "          see bellow if targets is not specified"
    echo ""
    echo "  Your env: nvidia_gpu=$nvidia_gpu, arch=$arch"
    echo "    default target=\"ros2 localization people people-nuc wireless server\" if nvidia_gpu=1, arch=x86_64"
    echo "    default target=\"ros2 localization people-nuc wireless server\"        if nvidia_gpu=0, arch=x86_64"
    echo "    default target=\"l4t\"                                                      if nvidia_gpu=0, arch=aarch64"
    echo ""
    echo "-h                    show this help"
    echo "-t <time_zone>        set time zone"
    echo "-n                    no cache option to build docker image"
    echo "-w                    build workspace only"
    echo "-i                    build image only"
    echo "-y                    no confirmation"
    echo "-u <uid>              replace uid"
}

# check if NVIDIA GPU is available (not including Jetson's Tegra GPU), does not consider GPU model
[[ ! $(lshw -json -C display 2> /dev/null | grep vendor) =~ NVIDIA ]]; nvidia_gpu=$?
arch=$(uname -m)
time_zone=$(cat /etc/timezone)

if [ $arch != "x86_64" ] && [ $arch != "aarch64" ]; then
    red "Unknown architecture: $arch"
    exit 1
fi

pwd=$(pwd)
scriptdir=$(dirname $0)
cd $scriptdir
scriptdir=$(pwd)
prefix=$(basename $scriptdir)
prefix_pb=${prefix}_

option="--progress=auto"
debug=0
build_ws=1
build_img=1
confirmation=1

export DOCKER_BUILDKIT=1
export COMPOSE_DOCKER_CLI_BUILD=1

while getopts "ht:Pndwiy" arg; do
    case $arg in
	h)
	    help
	    exit
	    ;;
	t)
	    time_zone=$OPTARG
	    ;;
	n)
	    option="$option --no-cache"
	    ;;
	d)
	    debug=1
	    export DOCKER_BUILDKIT=0
	    ;;
	w)
	    build_ws=1
	    build_img=0
	    ;;
	i)
	    build_ws=0
	    build_img=1
	    ;;
	y)
	    confirmation=0
	    ;;
    esac
done
shift $((OPTIND-1))
targets=$@

#
# specify default targets if targets is not specified
# if targets include all set all targets
#
if [ -z "$targets" ]; then
    if [ $nvidia_gpu -eq 1 ] && [ $arch = "x86_64" ]; then
	targets="ros2 localization people people-nuc wireless server gnss"
    elif [ $nvidia_gpu -eq 0 ] && [ $arch = "x86_64" ]; then
	targets="ros2 localization people-nuc wireless server gnss"
    elif [ $nvidia_gpu -eq 0 ] && [ $arch = "aarch64" ]; then
	targets="l4t"
    else
	red "Unknown combination, nvidia_gpu=$nvidia_gpu, arch=$arch"
	exit 1
    fi
elif [[ "$targets" =~ "all" ]]; then
    targets="ros2 localization people people-nuc wireless server l4t gnss"
fi

function check_to_proceed {
    if [[ "$targets" =~ "cuda" ]]; then
	REQUIRED_DRIVERV=450.80.02

	DRIVERV=`nvidia-smi | grep "Driver"`
	re=".*Driver Version: ([0-9\.]+) .*"
	if [[ $DRIVERV =~ $re ]];then
	    DRIVERV=${BASH_REMATCH[1]}
	    echo "NVDIA driver version $DRIVERV is found"
	    if $(dpkg --compare-versions $DRIVERV ge $REQUIRED_DRIVERV); then
		blue "Installed NVIDIA driver satisfies the required version $REQUIRED_DRIVERV"
	    else
		red "Installed NVIDIA driver does not satisfy the required version $REQUIRED_DRIVERV"
		if [ $confirmation -eq 1 ]; then
		    read -p "Press enter to continue or terminate"
		fi
	    fi
	fi
    fi

    if [[ "$targets" =~ "l4t" ]]; then
	if [ $arch = "aarch64" ]; then
	    blue "Building l4t image on aarch64 machine"
	elif [ $arch = "x86_64" ]; then
	    red "Building l4t image not on x86_64 machine"
	    if [ $(LANGUAGE=en_US apt list qemu 2> /dev/null | grep installed | wc -l) -eq 1 ]; then
		red "It takes time to build l4t image on emulator. Do you want to proceed?"
		if [ $confirmation -eq 1 ]; then
		    read -p "Press enter to continue or terminate"
		fi
	    else
		red "You need to install arm emurator to build l4t image on "
		exit 1
	    fi
	fi
    fi
}


function build_ros2_ws {
    debug_option=
    if [ $debug -eq 1 ]; then
	debug_option='-d'
    fi
    docker-compose run ros2 /home/developer/ros2_ws/script/cabot_ros2_build.sh $debug_option
    docker-compose -f docker-compose-bag.yaml run --rm bag bash -c "cd /home/developer/bag_ws && colcon build"
}

function build_localization_ws {
    docker-compose  run localization /launch.sh build
    if [ $? != 0 ]; then
	return $?
    fi
    docker-compose  -f docker-compose-mapping.yaml run localization /launch.sh build
}

function build_people_ws {
    docker-compose  run people /launch.sh build
}

function build_people-nuc_ws {
    docker-compose  -f docker-compose-common.yaml run people-nuc /launch.sh build
}

function build_l4t_ws {
    docker-compose  -f docker-compose-jetson.yaml run people-jetson /launch.sh build
}

function build_wireless_ws {
    : # nop
}

function build_gnss_ws {
    : # nop
}

function build_server_ws {
    : # nop
}

function build_ros2_image {
    local image=${prefix_pb}_jammy-realsense-humble-custom-mesa
    docker-compose build \
		   --build-arg FROM_IMAGE=$image \
		   --build-arg UID=$UID \
		   --build-arg TZ=$time_zone \
		   $option \
		   ros2
    if [ $? != 0 ]; then
	return 1
    fi
    docker-compose -f docker-compose-lint.yaml build \
		   --build-arg FROM_IMAGE=$image \
		   --build-arg UID=$UID \
		   --build-arg TZ=$time_zone \
		   $option \
		   lint
    if [ $? != 0 ]; then
	return 1
    fi
    docker-compose -f docker-compose-bag.yaml build \
		   --build-arg FROM_IMAGE=$image \
		   --build-arg UID=$UID \
		   --build-arg TZ=$time_zone \
		   $option \
		   bag
}

function build_localization_image {
    local image=${prefix_pb}_jammy-realsense-humble-custom-mesa
    docker-compose build \
		   --build-arg FROM_IMAGE=$image \
		   --build-arg ROS_DISTRO=humble \
		   --build-arg UID=$UID \
		   --build-arg TZ=$time_zone \
		   $option \
		   localization
    if [ $? != 0 ]; then
	return 1
    fi
    docker-compose -f docker-compose-mapping-post-process.yaml build \
		   --build-arg FROM_IMAGE=$image \
		   --build-arg UID=$UID \
		   --build-arg TZ=$time_zone \
		   $option \
		   post-process
}

function build_people_image {
    local image=${prefix_pb}_jammy-cuda11.7.1-cudnn8-devel-realsense-humble-custom-opencv-open3d-mesa
    docker-compose build \
		   --build-arg FROM_IMAGE=$image \
		   --build-arg UID=$UID \
		   --build-arg TZ=$time_zone \
		   $option \
		   people

    if [[ $? -ne 0 ]]; then
	return 1
    fi

    docker-compose -f docker-compose-rs3.yaml build \
		   --build-arg FROM_IMAGE=$image \
		   --build-arg UID=$UID \
		   --build-arg TZ=$time_zone \
		   $option \
		   people-rs1 people-rs2 people-rs3
}

function build_people-nuc_image {
    local image=${prefix_pb}_jammy-realsense-humble-custom-mesa
    docker-compose -f docker-compose-common.yaml build \
		   --build-arg FROM_IMAGE=$image \
		   --build-arg UID=$UID \
		   --build-arg TZ=$time_zone \
		   $option \
		   people-nuc
}

function build_l4t_image {
    local image=${prefix_pb}_l4t-opencv-humble-base-open3d
    export DOCKER_BUILDKIT=0
    docker-compose -f docker-compose-jetson.yaml build \
		   --build-arg FROM_IMAGE=$image \
		   --build-arg UID=$UID \
		   --build-arg TZ=$time_zone \
		   $option \
		   people-jetson
}

function build_wireless_image {
    local image=${prefix_pb}_jammy-realsense-humble-custom-mesa
    docker-compose  -f docker-compose-common.yaml build  \
		   --build-arg FROM_IMAGE=$image \
		   --build-arg ROS_DISTRO=humble \
		   --build-arg UID=$UID \
		   --build-arg TZ=$time_zone \
		   $option \
		   wifi_scan
    if [ $? != 0 ]; then
	return 1
    fi

    docker-compose  -f docker-compose-common.yaml build  \
		   --build-arg FROM_IMAGE=$image \
		   --build-arg ROS_DISTRO=humble \
		   --build-arg UID=$UID \
		   --build-arg TZ=$time_zone \
		   $option \
		   ble_scan
}

function build_gnss_image {
    local image=${prefix_pb}_jammy-realsense-humble-custom-mesa
    docker-compose -f docker-compose-gnss.yaml build  \
		   --build-arg FROM_IMAGE=$image \
		   --build-arg UID=$UID \
		   --build-arg TZ=$time_zone \
		   $option \
		   rtk_gnss
}

function build_server_image {
    docker-compose  -f docker-compose-server.yaml build  \
		   $option \
		   map_server
}

blue "Targets: $targets"
check_to_proceed
for target in $targets; do
    if [ $build_img -eq 1 ]; then
	blue "# Building $target image"
	eval "build_${target}_image"
	if [ $? != 0 ]; then
	    red "Got an error to build $target image"
	    echo "If you want to build image run ./prebuild-docker.sh first"
	    echo "If you want to pull image run ./manage-docker-image.sh (see README) and then run ./build-docker.sh with '-w' option"
	    exit 1
	fi
    fi
    if [ $build_ws -eq 1 ]; then
	blue "# Building $target workspace"
	eval "build_${target}_ws"
	if [ $? != 0 ]; then
	    red "Got an error to build $target workspace"
	    exit 1
	fi
    fi
done
