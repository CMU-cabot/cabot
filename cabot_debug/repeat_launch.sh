#!/bin/bash

stop_launch() {
    kill $launch_pid
    wait $launch_pid
    exit 0
}

trap 'stop_launch' SIGINT SIGTERM EXIT

function help() {
    echo "Usage:"
    echo "-h          show this help"
    echo "-f          set floor"
    echo "-t          repeat times"
    echo "-u          if usb recognition of realsense is not, exit"
    echo "-n          if you don't want to auto stop cabot.service and repeat, use this"
}

floor=0
repeat_times=2
usb_recognition=0
no_repeat=0
wait_map=0

CABOT_INITX=0
CABOT_INITY=0

pwd=$(pwd)
scriptdir=$(dirname $0)
cd $scriptdir
scriptdir=$(pwd)

cd $scriptdir/../
source ./.env

projectdir=$(pwd)
project=$(basename $projectdir)

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
shift $((OPTIND - 1))

source ./.env

pause() {
    trap true 1 2 3 15
    sleep infinity &
    wait $!
}

for i in $(eval echo {1..$repeat_times}); do
    if [[ -z $(docker ps -q -f "name=${project}-map_server*") ]]; then
        wait_map=1
    fi

    ./launch.sh -y &
    launch_pid=$!

    sleep 10

    if [[ $wait_map -eq 1 ]]; then
        echo "wait for map server to start"
        sleep 20
    fi

    ./cabot_debug/set-floor_and_estimate.sh -f $floor

    sleep 10

    if [[ $no_repeat -eq 1 ]]; then
        pause
    fi
    kill $launch_pid
    wait $launch_pid
    if [[ $CABOT_MODEL == "cabot2-gtm" ]] && [[ $usb_recognition -eq 1 ]]; then
        count=$(lsusb | grep 435 | wc -l)
        if [[ $count -le 1 ]]; then
            echo "realsense is down"
            break
        fi
    fi
done
