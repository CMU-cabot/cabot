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

## build base image

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
    echo "Usage"
    echo ""
    echo "-q                    quiet"
    echo "-h                    show this help"
    echo "-n                    nocache"
    echo "-t <time_zone>        set time zone"
    echo "-d                    debug without BUILDKIT"
    echo "-p                    project name"
}

pwd=`pwd`
prefix=`basename $pwd`
DIR=$pwd/prebuild
option="--progress=tty"
time_zone=`cat /etc/timezone`

export DOCKER_BUILDKIT=1
export DEBUG_FLAG="--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"
export UNDERLAY_MIXINS="rel-with-deb-info"
export OVERLAY_MIXINS="rel-with-deb-info"
debug_ros2="--build-arg DEBUG_FLAG"
debug_nav2="--build-arg UNDERLAY_MIXINS --build-arg OVERLAY_MIXINS"

CUDAV=11.1
CUDNNV=8
# See required NVIDIA driver version for CUDA here
# https://docs.nvidia.com/deploy/cuda-compatibility/
REQUIRED_DRIVERV=450.80.02

DRIVERV=`nvidia-smi | grep "Driver"`
re=".*Driver Version: ([0-9\.]+) .*"
if [[ $DRIVERV =~ $re ]];then
    DRIVERV=${BASH_REMATCH[1]}
    echo "NVDIA driver version $DRIVERV is found"
    if $(dpkg --compare-versions $DRIVERV ge $REQUIRED_DRIVERV); then
        echo "Installed NVIDIA driver satisfies the required version $REQUIRED_DRIVERV"
    else
        red "Installed NVIDIA driver does not satisfy the required version $REQUIRED_DRIVERV"
        exit
    fi
else
    red "NVIDIA driver is not found by nvidia-smi command"
    exit
fi

while getopts "hqnt:c:u:dp:" arg; do
    case $arg in
	h)
	    help
	    exit
	    ;;
	q)
	    option="-q $option"
	    ;;
	n)
	    option="--no-cache $option"
	    ;;
	t)
	    time_zone=$OPTARG
	    ;;
	d)
	    export DOCKER_BUILDKIT=0
	    ;;
	p)
	    prefix=$OPTARG
	    ;;
    esac
done

shift $((OPTIND-1))
target=$1
if [ "$target" = "" ]; then
    target=all
fi


blue "CUDA Vesrion: $CUDAV"
blue "cudnn Vesrion: $CUDNNV"
blue "TIME_ZONE=$time_zone"
read -p "Press enter to continue"


if [ $target = "ros" ] || [ $target = "all" ]; then
    echo ""
    blue "# build ${prefix}_galactic-ros-desktop-focal"
    pushd $DIR/galactic-desktop
    docker build -t ${prefix}_galactic-ros-desktop-focal \
	   --build-arg TZ=$time_zone \
	   $option $debug_ros2 \
	   .
    if [ $? -ne 0 ]; then
	red "failed to build galactic-desktop"
	exit
    fi
    popd
fi

if [ $target = "nav2" ] || [ $target = "all" ]; then
    echo ""
    blue "# build ${prefix}_navigation2"
    pushd $DIR/navigation2
    docker build -t ${prefix}_galactic-ros-desktop-nav2-focal \
	   --build-arg TZ=$time_zone \
	   --build-arg FROM_IMAGE=${prefix}_galactic-ros-desktop-focal \
	   $option $debug_nav2 \
	   .
    if [ $? -ne 0 ]; then
	red "failed to build navigation2"
	exit
    fi
    popd
fi

function build_cuda_ros_image() {
    local CUDAV=$1
    local CUDNNV=$2
    local UBUNTUV=$3
    local UBUNTU_DISTRO=$4
    local ROS_DISTRO=$5

    echo ""
    blue "# build ${prefix}_nvidia-cuda$CUDAV-cudnn$CUDNNV-devel-glvnd-runtime-ubuntu$UBUNTUV"
    pushd $DIR/opengl/glvnd/runtime/
    docker build -t ${prefix}_nvidia-cuda$CUDAV-cudnn$CUDNNV-devel-glvnd-runtime-ubuntu$UBUNTUV \
        --build-arg from=nvidia/cuda:$CUDAV-cudnn$CUDNNV-devel-ubuntu$UBUNTUV \
        --build-arg LIBGLVND_VERSION=v1.1.0 \
        $option \
        . 
    if [ $? -ne 0 ]; then
        red "failed to build glvnd"
        exit
    fi
    popd

    echo ""
    blue "# build ${prefix}_nvidia-cuda$CUDAV-cudnn$CUDNNV-devel-glvnd-runtime-ros-core-ubuntu$UBUNTUV"
    pushd $DIR/docker_images/ros/$ROS_DISTRO/ubuntu/$UBUNTU_DISTRO/ros-core/
    sed s/ubuntu:$UBUNTU_DISTRO/nvidia-cuda$CUDAV-cudnn$CUDNNV-devel-glvnd-runtime-ubuntu$UBUNTUV/ Dockerfile \
        > Dockerfile.CUDA$CUDAV &&\
        docker build -f Dockerfile.CUDA$CUDAV \
        -t ${prefix}_nvidia-cuda$CUDAV-cudnn$CUDNNV-devel-glvnd-runtime-ros-core-ubuntu$UBUNTUV \
        $option \
        .
    if [ $? -ne 0 ]; then
        red "failed to build ros-core"
        exit
    fi
    popd

    echo ""
    blue "# build ${prefix}_nvidia-cuda$CUDAV-cudnn$CUDNNV-devel-glvnd-runtime-ros-base-ubuntu$UBUNTUV"
    pushd $DIR/docker_images/ros/$ROS_DISTRO/ubuntu/$UBUNTU_DISTRO/ros-base/
    sed s/ros:$ROS_DISTRO-ros-core-$UBUNTU_DISTRO/nvidia-cuda$CUDAV-cudnn$CUDNNV-devel-glvnd-runtime-ros-core-ubuntu$UBUNTUV/ Dockerfile \
        > Dockerfile.CUDA$CUDAV &&\
        docker build -f Dockerfile.CUDA$CUDAV \
        -t ${prefix}_nvidia-cuda$CUDAV-cudnn$CUDNNV-devel-glvnd-runtime-ros-base-ubuntu$UBUNTUV \
        $option \
        .
    if [ $? -ne 0 ]; then
        red "failed to build ros-core"
        exit
    fi
    popd
}

function build_cuda_ros_realsense_image() {
    local CUDAV=$1
    local CUDNNV=$2
    local UBUNTUV=$3
    local UBUNTU_DISTRO=$4
    local ROS_DISTRO=$5

    blue "CUDA Vesrion: $CUDAV"
    blue "cudnn Vesrion: $CUDNNV"
    blue "Ubuntu Version: $UBUNTUV"

    echo ""
    blue "# build ${prefix}_nvidia-cuda$CUDAV-cudnn$CUDNNV-devel-glvnd-runtime-ros-base-realsense-ubuntu$UBUNTUV"
    #pushd $DIR/../  ## this is not good, because docker build context is too big
    pushd $DIR/realsense
    docker build -t ${prefix}_nvidia-cuda$CUDAV-cudnn$CUDNNV-devel-glvnd-runtime-ros-base-realsense-ubuntu$UBUNTUV \
        --build-arg from=nvidia-cuda$CUDAV-cudnn$CUDNNV-devel-glvnd-runtime-ros-base-ubuntu$UBUNTUV \
        --build-arg ROS_DISTRO=$ROS_DISTRO \
        --build-arg UBUNTU_DISTRO=$UBUNTU_DISTRO \
        $option \
        .
    if [ $? -ne 0 ]; then
        red "failed to build realsense"
        exit
    fi
    popd
}


if [ $target = "xenial" ] || [ $target = "all" ]; then
    UBUNTUV=16.04
    UBUNTU_DISTRO=xenial
    ROS_DISTRO=kinetic
    build_cuda_ros_image $CUDAV $CUDNNV $UBUNTUV $UBUNTU_DISTRO $ROS_DISTRO
fi

if [ $target = "focal" ] || [ $target = "all" ]; then
    UBUNTUV=20.04
    UBUNTU_DISTRO=focal
    ROS_DISTRO=noetic
    build_cuda_ros_image $CUDAV $CUDNNV $UBUNTUV $UBUNTU_DISTRO $ROS_DISTRO
    build_cuda_ros_realsense_image $CUDAV $CUDNNV $UBUNTUV $UBUNTU_DISTRO $ROS_DISTRO
fi
