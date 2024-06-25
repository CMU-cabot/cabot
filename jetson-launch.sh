#!/bin/bash

# Copyright (c) 2021  Carnegie Mellon University
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

trap ctrl_c INT QUIT TERM HUP

function ctrl_c() {
    IFS=' '
    for conf in $config; do
        IFS=':'
        items=($conf)
        ipaddress=${items[1]}
        if [ $verbose -eq 1 ]; then
            com="ssh -l $user $ipaddress \"cd cabot; docker-compose -f docker-compose-jetson.yaml down\""
        else
            com="ssh -l $user $ipaddress \"cd cabot; docker-compose -f docker-compose-jetson.yaml down\" > /dev/null 2>&1"
        fi
        if [ $verbose -eq 1 ]; then
            echo $com
        fi
        eval $com
    done

    for pid in ${pids[@]}; do
        if [ $verbose -eq 1 ]; then
            kill -s 2 $pid
        else
            kill -s 2 $pid > /dev/null 2>&1
        fi
    done
    for pid in ${pids[@]}; do
        if [ $verbose -eq 1 ]; then
            while kill -0 $pid; do
                snore 1
            done
        else
            while kill -0 $pid > /dev/null 2>&1; do
                snore 1
            done
        fi
    done
    exit
}
function snore()
{
    local IFS
    [[ -n "${_snore_fd:-}" ]] || exec {_snore_fd}<> <(:)
    read ${1:+-t "$1"} -u $_snore_fd || :
}

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
function help {
    echo "Usage"
    echo ""
    echo "-h                show this help"
    echo "-d                debug mode, show in xterm"
    echo "-t                test mode, launch roscore, publish transform"
    echo "-s                run on simulator"
    echo "-u <user>         user name"
    echo "-c <config>       configuration"
    echo "                  tracking  T:<ip>"
    echo "                  detection D:<ip>:<ns>"
    echo "-S <serial>       serial numbers of realsense cameras"
    echo "-f <fps>          RGB fps"
    echo "-p <fps>          depth fps"
    echo "-r <resolution>   resolution"
    echo "-o [1-3]          use specified opencv dnn implementation"
    echo "   1: python-opencv, 2: cpp-opencv-node, 3: cpp-opencv-nodelet"
    echo "-v                verbose"
    echo "-l                enable LiDAR post processing"
    echo "-N                disable people module"
    echo ""
    echo "$0 -u <user> -c \"T:192.168.1.50 D:192.168.1.51:rs1 \""
}
pids=()

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

user=
config=
serial_nums=
rgb_fps=
depth_fps=
resolution=
testmode=0
testopt=
simulator=0
command="bash -c \""
commandpost="\"&"
verbose=0
process_lidar=0
disable_people=0

while getopts "hdtsu:c:S:f:p:r:o:vlN" arg; do
    case $arg in
        h)
            help
            exit
            ;;
        d)
            command="setsid xterm -fa 'Monospace' -fs 11 -e \""
            commandpost=";read\"&"
            ;;
        t)
            testmode=1
            testopt="-t"
            ;;
        s)
            simulator=1
            ;;
        u)
            user=$OPTARG
            ;;
        c)
            config=$OPTARG
            ;;
        S)
            serial_nums=$OPTARG
            ;;
        f)
            rgb_fps=$OPTARG
            ;;
        p)
            depth_fps=$OPTARG
            ;;
        r)
            resolution=$OPTARG
            ;;
        o)
            opencv_dnn_ver=$OPTARG
            ;;
        v)
            verbose=1
            ;;
        l)
            process_lidar=1
            ;;
        N)
            disable_people=1
            ;;
    esac
done

error=0
if [ -z "$user" ]; then
    err "Please specify user to login"
    error=1
fi

if [ -z "$config" ]; then
    err "Config is not specified"
    error=1
fi

if [ -z "$serial_nums" ]; then
    err "RealSense serial number is not specified"
    error=1
fi

if [ -z "$opencv_dnn_ver" ]; then
    err "opencv dnn implementation is not specified"
    error=1
fi

if [ $error -eq 1 ]; then
    help
    exit
fi

if [ $testmode -eq 1 ]; then
    if [ $verbose -eq 1 ]; then
        com="$command roscore $commandpost"
    else
        com="$command roscore > /dev/null $commandpost"
    fi

    eval $com
    pids+=($!)
fi

declare -A serial_array
for serial_num in $serial_nums; do
    OLDIFS=$IFS
    IFS=':'
    items=($serial_num)
    serial_array[${items[0]}]=${items[1]}
    IFS=$OLDIFS
done

for conf in $config; do
    OLDIFS=$IFS
    IFS=':'
    items=($conf)
    mode=${items[0]}
    ipaddress=${items[1]}
    name=${items[2]}
    IFS=$OLDIFS

    if [ $verbose -eq 1 ]; then
	echo $conf $mode $ipaddress $name
    fi

    if [ "$mode" == 'D' ]; then
        if [ -z $name ]; then
            err "You need to specify camera namespace"
            exit
        fi
        if [ -z ${serial_array[$name]} ]; then
            err "You need to specify RealSense serial number for camera $name"
            exit
        fi

        blue "launch detect @ $ipaddress with $name"

        serial=${serial_array[$name]}

        camopt="-r -D -v $opencv_dnn_ver"
        if [ $simulator -eq 1 ]; then
            camopt="-s -D -v $opencv_dnn_ver"
        fi

        if [ $verbose -eq 1 ]; then
            com="$command ssh -l $user $ipaddress \
\\\"cd cabot; \
export CABOT_DETECT_PEOPLE_CONF_THRES='$CABOT_DETECT_PEOPLE_CONF_THRES'; \
export CABOT_DETECT_PEOPLE_FPS='$CABOT_DETECT_PEOPLE_FPS'; \
export CABOT_ENABLE_LIDAR_PROCESSING=${process_lidar}; \
export CABOT_DISABLE_PEOPLE=${disable_people}; \
docker-compose -f docker-compose-jetson.yaml run --rm people-jetson sudo /resetrs.sh \
$serial; \
docker-compose -f docker-compose-jetson.yaml run --rm people-jetson /launch.sh \
$testopt \
$camopt \
-N ${name} \
-F ${rgb_fps} \
-P ${depth_fps} \
-R ${resolution} \
-f ${name}_link \
-S ${serial} \\\" $commandpost"
        else
            com="$command ssh -l $user $ipaddress \
\\\"cd cabot; \
export CABOT_DETECT_PEOPLE_CONF_THRES='$CABOT_DETECT_PEOPLE_CONF_THRES'; \
export CABOT_DETECT_PEOPLE_FPS='$CABOT_DETECT_PEOPLE_FPS'; \
export CABOT_ENABLE_LIDAR_PROCESSING=${process_lidar}; \
export CABOT_DISABLE_PEOPLE=${disable_people}; \
docker-compose -f docker-compose-jetson.yaml run --rm people-jetson sudo /resetrs.sh \
$serial; \
docker-compose -f docker-compose-jetson.yaml run --rm people-jetson /launch.sh \
$testopt \
$camopt \
-N ${name} \
-F ${rgb_fps} \
-P ${depth_fps} \
-R ${resolution} \
-f ${name}_link \
-S ${serial} \\\" > /dev/null 2>&1 $commandpost"
        fi
    elif [ "$mode" == 'T' ]; then
        blue "launch tracking @ $ipaddress"

        if [ $verbose -eq 1 ]; then
            com="$command ssh -l $user $ipaddress \
\\\"cd cabot; \
export CABOT_ENABLE_LIDAR_PROCESSING=${process_lidar}; \
export CABOT_DISABLE_PEOPLE=${disable_people}; \
docker-compose -f docker-compose-jetson.yaml run --rm people-jetson /launch.sh \
-K \\\" $commandpost"
        else
            com="$command ssh -l $user $ipaddress \
\\\"cd cabot; \
export CABOT_ENABLE_LIDAR_PROCESSING=${process_lidar}; \
export CABOT_DISABLE_PEOPLE=${disable_people}; \
docker-compose -f docker-compose-jetson.yaml run --rm people-jetson /launch.sh \
-K \\\" > /dev/null 2>&1 $commandpost"
        fi
    else
	err "Unknown mode: $mode"
	exit
    fi

    if [ $verbose -eq 1 ]; then
        echo $com
    fi
    eval $com

    pids+=($!)
done

while [ 1 -eq 1 ];
do
    snore 1
done
