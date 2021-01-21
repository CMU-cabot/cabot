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
}

DIR=prebuild
option=
time_zone=`cat /etc/timezone`

export DEBUG_FLAG="--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"
export UNDERLAY_MIXINS="rel-with-deb-info"
export OVERLAY_MIXINS="rel-with-deb-info"
debug_ros2="--build-arg DEBUG_FLAG"
debug_nav2="--build-arg UNDERLAY_MIXINS --build-arg OVERLAY_MIXINS"


while getopts "hqnt:" arg; do
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
    esac
done

blue "TIME_ZONE=$time_zone"

echo ""
blue "# build foxy-ros-desktop-focal"
pushd $DIR/foxy-desktop-src
docker build -t foxy-ros-desktop-focal \
       --build-arg TZ=$time_zone \
       $option $debug_ros2 \
       .
if [ $? -ne 0 ]; then
    red "failed to build foxy-desktop"
    exit
fi
popd

echo ""
blue "# build navigation2"
pushd $DIR/navigation2
docker build -t foxy-ros-desktop-nav2-focal \
       --build-arg TZ=$time_zone \
       --build-arg FROM_IMAGE=foxy-ros-desktop-focal \
       $option $debug_nav2 \
       . 
popd
