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
    echo "$0 [<option>] [<targets>]"
    echo ""
    echo "targets : all: all targets"
    echo "          ros2: build ROS2 humble with Nav2"
    echo "          cuda: build ROS2 humble on CUDA"
    echo "          l4t:  build ROS1 melodic for jeston"
    echo "          see bellow if targets is not specified"
    echo ""
    echo "  Your env: nvidia_gpu=$nvidia_gpu, arch=$arch"
    echo "    default target=\"ros2 cuda\" if nvidia_gpu=1, arch=x86_64"
    echo "    default target=\"ros2\"      if nvidia_gpu=0, arch=x86_64"
    echo "    default target=\"l4t\"            if nvidia_gpu=0, arch=aarch64"
    echo ""
    echo "-h                    show this help"
    echo "-q                    quiet option for docker build"
    echo "-n                    nocache option for docker build"
    echo "-t <time_zone>        set time zone (default=$time_zone, your local time zone)"
    echo "-d                    debug without BUILDKIT"
    echo "-y                    no confirmation"
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
prefix=$(basename $scriptdir)_

prebuild_dir=$scriptdir/docker/prebuild
option="--progress=auto"
confirmation=1

export DOCKER_BUILDKIT=1
export DEBUG_FLAG="--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"
export UNDERLAY_MIXINS="rel-with-deb-info"
export OVERLAY_MIXINS="rel-with-deb-info"
debug_ros2="--build-arg DEBUG_FLAG"
debug_nav2="--build-arg UNDERLAY_MIXINS --build-arg OVERLAY_MIXINS"

while getopts "hqnt:c:u:dy" arg; do
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
	targets="ros2 cuda"
    elif [ $nvidia_gpu -eq 0 ] && [ $arch = "x86_64" ]; then
	targets="ros2"
    elif [ $nvidia_gpu -eq 0 ] && [ $arch = "aarch64" ]; then
	targets="l4t"
    else
	red "Unknown combination, nvidia_gpu=$nvidia_gpu, arch=$arch"
	exit 1
    fi
elif [[ "$targets" =~ "all" ]]; then
    targets="ros2 cuda l4t"
fi

CUDAV=11.7.1
CUDNNV=8
ROS2_UBUNTUV=22.04
UBUNTU_DISTRO=focal
ROS2_UBUNTU_DISTRO=jammy
ROS_DISTRO=noetic
ROS2_DISTRO=humble

#ros2       - jammy-humble-ros-desktop-nav2-vcs-mesa-humble-custom   nvidia, mesa
#l10n       - focal-ros-noetic-mesa                    nvidia, mesa
#people     - jammy-cuda-humble-base                   nvidia
#people-nuc - jammy-humble-ros-desktop-nav2-vcs-mesa   nvidia, mesa
#ble_scan   - jammy-humble-ros-desktop-nav2-vcs-mesa   nvidia, mesa
#people-l4t - l4t-ros-desktop

#focal-noetic-base-mesa
#jummy-humble-desktop-nav2-vcs-mesa
#jummy-cuda-humble-base
#l4t-melodic-py3-desktop

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
	    if [ $(apt list qemu 2> /dev/null | grep installed | wc -l) -eq 1 ]; then
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


function build_ros_base_image {
    local FROM_IMAGE=$1
    local IMAGE_TAG_PREFIX=$2
    local UBUNTU_DISTRO=$3
    local ROS_DISTRO=$4
    local ROS_COMPONENT=$5
    local -n IMAGE_TAG=$6

    echo ""
    IMAGE_TAG=$IMAGE_TAG_PREFIX-$ROS_DISTRO
    blue "## build $IMAGE_TAG"
    pushd $prebuild_dir/docker_images/ros/$ROS_DISTRO/ubuntu/$UBUNTU_DISTRO/ros-core/

    sed s=FROM.*=FROM\ $FROM_IMAGE= Dockerfile > Dockerfile.temp && \
        docker build -f Dockerfile.temp -t $IMAGE_TAG $option .
    if [ $? -ne 0 ]; then
        red "failed to build $IMAGE_TAG"
        exit 1
    fi
    popd

    echo ""
    FROM_IMAGE=$IMAGE_TAG
    IMAGE_TAG=$IMAGE_TAG_PREFIX-$ROS_DISTRO-base
    blue "## build $IMAGE_TAG"
    pushd $prebuild_dir/docker_images/ros/$ROS_DISTRO/ubuntu/$UBUNTU_DISTRO/ros-base/
    sed s=FROM.*=FROM\ $FROM_IMAGE= Dockerfile > Dockerfile.temp && \
        docker build -f Dockerfile.temp -t $IMAGE_TAG $option .
    if [ $? -ne 0 ]; then
        red "failed to build $IMAGE_TAG"
        exit 1
    fi
    popd

    if [[ $ROS_COMPONENT = "ros-base" ]]; then
	returnn
    fi

    echo ""
    IMAGE_TAG=$IMAGE_TAG_PREFIX-$ROS_DISTRO-desktop
    blue "## build $IMAGE_TAG"
    pushd $prebuild_dir/docker_images/ros/$ROS_DISTRO/ubuntu/$UBUNTU_DISTRO/desktop/
    sed s=FROM.*=FROM\ $FROM_IMAGE= Dockerfile > Dockerfile.temp && \
        docker build -f Dockerfile.temp -t $IMAGE_TAG $option .
    if [ $? -ne 0 ]; then
        red "failed to build $IMAGE_TAG"
        exit 1
    fi
    popd
}

function prebuild {
    local FROM_IMAGE=$1
    local IMAGE_BASE_NAME=$2
    local IMAGE_DIR=$3
    local -n IMAGE_TAG=$4         # output variable name

    IMAGE_TAG=$IMAGE_BASE_NAME-$IMAGE_DIR

    pushd $prebuild_dir/$IMAGE_DIR
    blue "## build $IMAGE_TAG"
    docker build -t $IMAGE_TAG \
	   --file Dockerfile \
	   --build-arg TZ=$time_zone \
	   --build-arg FROM_IMAGE=$FROM_IMAGE \
	   . && popd
}

function build_ros2 {
    blue "- UBUNTU_DISTRO=$ROS2_UBUNTU_DISTRO"
    blue "- ROS2_DISTRO=$ROS2_DISTRO"
    blue "- TIME_ZONE=$time_zone"

    base_image=ubuntu:$ROS2_UBUNTU_DISTRO
    base_name=${prefix}_${ROS2_UBUNTU_DISTRO}
    prebuild $base_image $base_name realsense image_tag
    if [ $? -ne 0 ]; then
	return 1
    fi

    base_name=$image_tag
    build_ros_base_image $image_tag $image_tag $ROS2_UBUNTU_DISTRO $ROS2_DISTRO desktop image_tag
    if [ $? -ne 0 ]; then
	red "failed to build $name1"
	exit 1
    fi

    prebuild $image_tag $base_name ${ROS2_DISTRO}-custom image_tag
    if [ $? -ne 0 ]; then
	red "failed to build $image_tag"
	return 1
    fi

    prebuild $image_tag $image_tag mesa image_tag
    if [ $? -ne 0 ]; then
	red "failed to build $image_tag"
	return 1
    fi
}

function build_cuda {
    blue "- CUDAV=$CUDAV"
    blue "- CUDNNV=$CUDNNV"
    blue "- UBUNTUV=$ROS2_UBUNTUV"
    blue "- UBUNTU_DISTRO=$ROS2_UBUNTU_DISTRO"
    blue "- ROS_DISTRO=$ROS2_DISTRO"

    cuda_base=nvidia/cuda:$CUDAV-cudnn$CUDNNV-devel-ubuntu$ROS2_UBUNTUV
    base_name=${prefix}_$ROS2_UBUNTU_DISTRO-cuda$CUDAV-cudnn$CUDNNV-devel
    prebuild $cuda_base $base_name realsense image_tag
    if [ $? -ne 0 ]; then
	return 1
    fi

    base_name=${image_tag}
    blue "## build $base_name-$ROS2_DISTRO-desktop"
    build_ros_base_image $image_tag $image_tag $ROS2_UBUNTU_DISTRO $ROS2_DISTRO desktop image_tag

    prebuild $image_tag $base_name ${ROS2_DISTRO}-custom image_tag
    if [ $? -ne 0 ]; then
	return 1
    fi

    prebuild $image_tag $image_tag opencv image_tag
    if [ $? -ne 0 ]; then
	red "failed to build $image_tag"
	return 1
    fi

    prebuild $image_tag $image_tag open3d image_tag
    if [ $? -ne 0 ]; then
	red "failed to build $image_tag"
	return 1
    fi

    prebuild $image_tag $image_tag mesa image_tag
    if [ $? -ne 0 ]; then
	red "failed to build $image_tag"
	return 1
    fi
}

function build_l4t {
    export DOCKER_BUILDKIT=0
    L4T_IMAGE="nvcr.io/nvidia/l4t-base:r35.1.0"

    echo ""
    local name1=${prefix}_l4t-opencv
    blue "# build ${prefix}_l4t-opencv"
    pushd $prebuild_dir/cv
    docker build -f Dockerfile.opencv-limited.jetson -t $name1 \
	   --build-arg FROM_IMAGE=$L4T_IMAGE \
	   $option \
	   .
    if [ $? -ne 0 ]; then
	red "failed to build $name1"
	exit 1
    fi
    popd

    echo ""
    local name2=${prefix}_l4t-opencv-humble-base
    blue "# build ${prefix}_l4t-opencv-humble-base"
    pushd $prebuild_dir/jetson-humble-base-src
    docker build -t $name2 \
	   --build-arg FROM_IMAGE=$name1 \
	   $option \
	   .
    if [ $? -ne 0 ]; then
	red "failed to build $name2"
	exit 1
    fi
    popd

    echo ""
    local name3=${prefix}_l4t-opencv-humble-base-open3d
    blue "# build ${prefix}_l4t-opencv-humble-base-open3d"
    pushd $prebuild_dir/cv
    docker build -f Dockerfile.open3d.jetson -t $name3 \
	   --build-arg FROM_IMAGE=$name2 \
	   $option \
	   .
    if [ $? -ne 0 ]; then
	red "failed to build $name3"
	exit 1
    fi
    popd
}

blue "Targets: $targets"
check_to_proceed
for target in $targets; do
    blue "# Building $target"
    eval "build_$target"
    if [[ $? -ne 0 ]]; then
	red "failed to build $target"
	break
    fi
done
