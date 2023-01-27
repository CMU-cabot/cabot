#!/bin/bash

function help()
{
    echo "Usage:"
    echo "-h          show this help"
    echo "-r          specify which recording file is going to be processed (required, please specify a svo file)"
    echo "-q          query cartographer result for the specified recording file"
    echo "-c          copy bag file associated with the recording file into recording dir"
    echo "-t          test with the recording file"
    echo ""
    echo "example usage:"
    echo "1. $0 -r recordings/cabot_20xx-xx-xx-xx-xx-xx_data3d.svo (wait until the process is finished)"
    echo "2. $0 -q -r recordings/cabot_20xx-xx-xx-xx-xx-xx_data3d.svo (run in another terminal before closing the first one)"
    echo "3. $0 -c -r recordings/cabot_20xx-xx-xx-xx-xx-xx_data3d.svo (copy associated bag file next to the svo file)"
}

file=
query=0
copy_bagfile=0
test=0

while getopts "hr:qtc" arg; do
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
	t)
	    test=1
	    ;;
	c)
	    copy_bagfile=1
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

bagfile_docker="/home/developer/.ros/log/$base/ros1_topics.bag"
bagfile_host="./docker/home/.ros/log/$base/ros1_topics.bag"

if [[ $copy_bagfile -eq 1 ]]; then
    cp $bagfile_host ${dir}/${base}.bag
elif [[ $query -eq 1 ]]; then
    docker-compose -f docker-compose-mapping.yaml run localization bash -c "source devel/setup.bash; rosservice call /trajectory_query \"trajectory_id: 1\" > ${base}-trajectory.yaml"
    mv docker/home/loc_ws/$base-trajectory.yaml $dir
else
    if [[ $test -eq 1 ]]; then
	docker-compose -f docker-compose-mapping.yaml run localization ./tbd-process.sh -t $bagfile_docker
    else
	if [[ ! -e $bagfile_host ]]; then
	    echo "there is no ${bagfile_host}"
	    if [[ -e ${bagfile_host}.active ]]; then
		echo "there is unindexed bag file, will reindex ${bagfile_host}.active"
		rosbag info ${bagfile_host}.active
		if [[ $? -ne 0 ]]; then
		    rosbag reindex ${bagfile_host}.active
		fi
		mv ${bagfile_host}.active ${bagfile_host}
	    fi
	fi
	if [[ -e $bagfile_host ]]; then
	    docker-compose -f docker-compose-mapping.yaml run localization ./tbd-process.sh $bagfile_docker
	fi
    fi
fi
