#!/bin/bash

function help {
    echo "Usage: $0 [<options>] <bag_file_path>"
    echo "-h          show this help"
    echo "-r <rate>   play bag rate"
    echo "-s <offset> play bag offset, bigger than 0"
    echo "-c          rotate camera image based on TF (only for rs3 settings)"
    echo "-q          open with rqt_bag"
    echo "-R <robot>  specify the robot model if necessary (e.g. cabot3-k4)"
}

# change directory to where this script exists
pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

cd $scriptdir/../

rate=1.0
start=0.01
rqt_bag=0
robot=
rotate_camera=0
while getopts "hdr:s:qR:c" arg; do
    case $arg in
        h)
            help
            exit
            ;;
        r)
            rate=$OPTARG
            ;;
        s)
            start=$OPTARG
            ;;
        q)
            rqt_bag=1
            ;;
        R)
            robot="-R $OPTARG"
            ;;
        c)
            rotate_camera=1
            ;;
    esac
done
shift $((OPTIND-1))

bag=$1

if [[ -z $bag ]]; then
    help
    exit 1
fi

if [[ "$bag" != /* && "$bag" != .* ]]; then
    bag="./$bag"
fi

if [[ -z $bag ]]; then
    echo "Usage: $0 <bag_file>"
    exit 1
fi

echo $bag

if [[ $rotate_camera -eq 1 ]]; then
    docker compose -f docker-compose-rotate-rs.yaml up -d
fi

if [[ $rqt_bag -eq 1 ]]; then
    com="CABOT_BAG_MOUNT=$bag docker compose -f docker-compose-bag.yaml run --rm bag-dev ros2 run rqt_bag rqt_bag /ros2_topics"
else
    com="CABOT_BAG_MOUNT=$bag docker compose -f docker-compose-bag.yaml run --rm bag-dev /launch.sh play -r $rate -s $start $robot /ros2_topics"
fi
echo $com
eval $com

if [[ $rotate_camera -eq 1 ]]; then
    docker compose -f docker-compose-rotate-rs.yaml down
fi
