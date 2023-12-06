#!/bin/bash

trap 'systemctl --user stop cabot.service; exit' SIGINT


function help()
{
    echo "Usage:"
    echo "-h          show this help"
    echo "-f          set floor"
    echo "-t          repeat times"
    echo "-u          if usb recognition of realsense is not, exit"
    echo "-n          if you don't want to auto stop cabot.service and repeat, use this"
}

floor=0
repeat_times=20
usb_recognition=0
no_repeat=0

CABOT_INITX=0
CABOT_INITY=0

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

cd $scriptdir/../
source ./.env

while getopts "hf:t:un" arg; do
    case $arg in
        h)
            help
            exit
            ;;
        f)
            floor=$OPTARG
            ;;
	t)
	    repeat_times=$OPTARG
	    ;;
	u)
	    usb_recognition=1
	    ;;
	n)
	    no_repeat=1
	    repeat_times=1
	    ;;
    esac
done
shift $((OPTIND-1))

source ./.env


pause(){
    trap true 1 2 3 15
    sleep infinity&
    wait $!
}


for i in `eval echo {1..$repeat_times}`; do
    systemctl --user start cabot.service
    sleep 10
    if [[ $i -eq 1 ]]; then
	echo "wait for map server to start"
	sleep 20
    fi
    container_id=`docker ps | grep localization | cut -d " " -f 1`
    docker exec $container_id bash -c 'source install/setup.bash; ros2 service call /set_current_floor mf_localization_msgs/srv/MFSetInt "{data: '$floor'}"'
    sleep 10
    ros2 topic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped '{ header: {frame_id: "map_global"}, pose: { pose: {position: {x: '$CABOT_INITX', y: '$CABOT_INITY'}}}}'
    sleep 180
    if [[ $no_repeat -eq 1 ]]; then
	pause
    fi
    systemctl --user stop cabot.service
    if [[ $CABOT_MODEL == "cabot2-gtm" ]] && [[ $usb_recognition -eq 1 ]]; then
        count=`lsusb | grep 435 | wc -l`
        if [[ $count -le 1 ]]; then
	    echo "realsense is down"
            break
        fi
    fi
done
