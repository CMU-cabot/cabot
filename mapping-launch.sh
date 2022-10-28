#!/bin/bash

# Copyright (c) 2022  Carnegie Mellon University
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

function err {
    >&2 red "[ERROR] "$@
}
function red {
    echo -en "\033[31m"  ## red
    echo $@
    echo -en "\033[0m"  ## reset color
}
function blue {
    echo -en "\033[36m"  ## blue
    echo $@
    echo -en "\033[0m"  ## reset color
}
function snore()
{
    local IFS
    [[ -n "${_snore_fd:-}" ]] || exec {_snore_fd}<> <(:)
    read ${1:+-t "$1"} -u $_snore_fd || :
}
function help()
{
    echo "Usage:"
    echo " this program runs recording for mapping"
    echo ""
    echo "-h          show this help"
    echo "-c          run realtime cartographer"
    echo "-a          use arduino for IMU topic"
    echo "-x          use xsens for IMU topic"
    echo "-o <name>   output prefix (default=mapping)"
    echo "-p <file>   post process the recorded bag"
    echo "-w          do not wait when rosbag play is finished"
    echo "-n          not use cached result for post processing"
    echo "-r <rate>   rosbag play rate for cartographer (default=1.0)"
    echo "-R <rate>   rosbag play rate for converting pointcloud2 to laserscan (default=1.0)"
}

OUTPUT_PREFIX=${OUTPUT_PREFIX:=mapping}
RUN_CARTOGRAPHER=false
USE_ARDUINO=false
USE_XSENS=false
USE_VELODYNE=true
PLAYBAG_RATE_CARTOGRAPHER=1.0
PLAYBAG_RATE_PC2_CONVERT=1.0

post_process=
wait_when_rosbag_finish=1
no_cache=0

while getopts "hcaxo:p:wnr:R:" arg; do
    case $arg in
        h)
            help
            exit
            ;;
	c)
	    RUN_CARTOGRAPHER=true
	    ;;
	a)
	    USE_ARDUINO=true
	    ;;
	x)
	    USE_XSENS=true
	    ;;
	o)
	    OUTPUT_PREFIX=$OPTARG
	    ;;
	p)
	    post_process=$(realpath $OPTARG)
	    ;;
	w)
	    wait_when_rosbag_finish=0
	    ;;
	n)
	    no_cache=1
	    ;;
	r)
	    PLAYBAG_RATE_CARTOGRAPHER=$OPTARG
	    ;;
	R)
	    PLAYBAG_RATE_PC2_CONVERT=$OPTARG
	    ;;
    esac
done
shift $((OPTIND-1))

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

if [[ -n $post_process ]]; then
    if [[ ! -e $post_process ]]; then
	err "could not find $post_process file"
	exit
    fi
    blue "processing $post_process"
    post_process_dir=$(dirname $post_process)
    post_process_name=$(basename $post_process)

    mkdir -p $scriptdir/docker/home/post_process
    if [[ $no_cache -eq 1 ]]; then
	rm $scriptdir/docker/home/post_process/${post_process_name}*
    fi
    cp $post_process $scriptdir/docker/home/post_process/
    
    QUIT_WHEN_ROSBAG_FINISH=true
    if [[ $wait_when_rosbag_finish -eq 1 ]]; then
	QUIT_WHEN_ROSBAG_FINISH=false
    fi
    export QUIT_WHEN_ROSBAG_FINISH
    export BAG_FILENAME=${post_process_name%.*}
    export PLAYBAG_RATE_CARTOGRAPHER
    export PLAYBAG_RATE_PC2_CONVERT
    docker-compose -f docker-compose-mapping-post-process.yaml run post-process
    exit
fi

echo "OUTPUT_PREFIX=$OUTPUT_PREFIX"
echo "RUN_CARTOGRAPHER=$RUN_CARTOGRAPHER"
echo "USE_ARDUINO=$USE_ARDUINO"
echo "USE_XSENS=$USE_XSENS"
echo "USE_VELODYNE=$USE_VELODYNE"

cd $scriptdir
export OUTPUT_PREFIX=$OUTPUT_PREFIX
export RUN_CARTOGRAPHER=$RUN_CARTOGRAPHER
export USE_ARDUINO=$USE_ARDUINO
export USE_XSENS=$USE_XSENS
export USE_VELODYNE=$USE_VELODYNE
docker-compose -f docker-compose-mapping.yaml up
