#!/bin/bash

function help {
    echo "Usage:"
    echo "-h          show this help"
    echo "-r <rate>   play bag rate"
    echo "-s <offset> play bag offset, bigger than 0"
}

# change directory to where this script exists
pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

cd $scriptdir/../

rate=1.0
start=0.01
while getopts "hr:s:" arg; do
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
    esac
done
shift $((OPTIND-1))

bag=$1

if [[ -z $bag ]]; then
    echo "Usage: $0 <bag_file>"
    exit 1
fi

echo $bag

CABOT_BAG_MOUNT=$bag docker-compose -f docker-compose-bag.yaml run --rm bag /launch-bag.sh play -r $rate -s $start /ros2_topics
