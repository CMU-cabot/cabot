#!/bin/bash

function help {
    echo "Usage: $0 [<options>] <bag_file_path>"
    echo "-h          show this help"
    echo "-r <rate>   play bag rate"
    echo "-s <offset> play bag offset, bigger than 0"
    echo "-q          open with rqt_bag"
}

# change directory to where this script exists
scriptdir=$(dirname "$0")
cd "$scriptdir" || exit
scriptdir=$(pwd)

cd "$scriptdir/../" || exit

rate=1.0
start=0.01
rqt_bag=0
while getopts "hr:s:q" arg; do
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
    *) ;;
    esac
done
shift $((OPTIND - 1))

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

echo "$bag"

if [[ $rqt_bag -eq 1 ]]; then
    com="CABOT_BAG_MOUNT=$bag docker compose -f docker-compose-bag.yaml run --rm bag ros2 run rqt_bag rqt_bag /ros2_topics"
else
    com="CABOT_BAG_MOUNT=$bag docker compose -f docker-compose-bag.yaml run --rm bag /launch.sh play -r $rate -s $start /ros2_topics"
fi
echo "$com"
eval "$com"
