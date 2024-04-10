#!/bin/bash

function help() {
    echo "Usage:"
    echo "-h          show this help"
    echo "-f          set floor"
}

floor=0
CABOT_INITX=0
CABOT_INITY=0

pwd=$(pwd)
scriptdir=$(dirname $0)
cd $scriptdir
scriptdir=$(pwd)

cd $scriptdir/../
source ./.env

while getopts "hf:" arg; do
    case $arg in
    h)
        help
        exit
        ;;
    f)
        floor=$OPTARG
        ;;
    esac
done
shift $((OPTIND - 1))

container_id=$(docker ps | grep localization | cut -d " " -f 1)
docker exec $container_id bash -c 'source install/setup.bash; ros2 service call /set_current_floor mf_localization_msgs/srv/MFSetInt "{data: '$floor'}"'
sleep 3
ros2 topic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped '{ header: {frame_id: "map_global"}, pose: { pose: {position: {x: '$CABOT_INITX', y: '$CABOT_INITY'}}}}'
