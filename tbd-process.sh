#!/bin/bash

function help()
{
    echo "Usage:"
    echo "-h          show this help"
    echo "-r          recording file"
}

file=
query=0

while getopts "hr:q" arg; do
    case $arg in
        h)
            help
            exit
            ;;
        r)
            file=$OPTARG
            ;;
	q)
	    query=1
	    ;;
    esac
done
shift $((OPTIND-1))

if [[ $file == "" ]]; then
    echo "You need to specify a reconrding file -r <filepath>"
    exit
fi

re=".*/(cabot_[0-9\-]+)_.*"
if [[ $file =~ $re ]]; then
    base=${BASH_REMATCH[1]};
fi
dir=$(dirname $file)

if [[ $query -eq 1 ]]; then
    docker-compose -f docker-compose-mapping.yaml run localization bash -c "source devel/setup.bash; rosservice call /trajectory_query \"trajectory_id: 1\" > ${base}-trajectory.yaml"
    mv docker/home/loc_ws/$base-trajectory.yaml $dir
else
    bagfile="/home/developer/.ros/log/$base/ros1_topics.bag"
    
    
    docker-compose -f docker-compose-mapping.yaml run localization ./tbd-process.sh $bagfile
fi
