#!/bin/bash

function red {
    echo -en "\033[31m"  ## red
    echo $1
    echo -en "\033[0m"  ## reset color
}
function blue {
    echo -en "\033[36m"  ## blue
    echo $1
    echo -en "\033[0m"  ## reset color
}
function help {
    echo "Usage: $0 <option>"
    echo ""
    echo "-h                    show this help"
    echo "-t <tag_name>         set docker image tag name"
    echo "-n                    not copy"
}

tag=prod
not_copy=0
while getopts "ht:n" arg; do
    case $arg in
	h)
	    help
	    exit
	    ;;
	t)
	    tag=$OPTARG
	    ;;
	n)
	    not_copy=1
	    ;;
    esac
done
shift $((OPTIND-1))
target=$1

if [ -z $target ]; then
    target=all
fi

if [ $target == "l4t" ] || [ $target == "all" ]; then
    if [ $not_copy -eq 0 ]; then
       blue "copy package files"
       rm -rf ./docker/people/src/*
       cp -r ./cabot_sites ./docker/people/src/
       cp -r ./mf_localization_msgs ./docker/people/src/
       cp -r ./cabot_people ./docker/people/src/
       cp -r ./predict_people_py ./docker/people/src/
       cp -r ./queue_people_py ./docker/people/src/
       cp -r ./track_people_py ./docker/people/src/
       cp -r ./track_people_cpp ./docker/people/src/
       cp -r ./queue_utils_py ./docker/people/src/
       cp -r ./docker/bridge/ros1/queue_msgs ./docker/people/src/
       cp -r ./docker/bridge/ros1/people/people_msgs ./docker/people/src/
       cp -r ./docker/home/people_ws/src/realsense_ros ./docker/people/src/
       cp -r ./docker/home/people_ws/src/image_common ./docker/people/src/
       cp -r ./docker/home/people_ws/src/vision_opencv ./docker/people/src/
       cp -r ./docker/home/people_ws/src/ddynamic_reconfigure ./docker/people/src/
       cp -r ./docker/home/people_ws/src/image_transport_plugins ./docker/people/src/
       blue "deleting unused files"
       pushd docker/people/src
       find . -name ".git" -exec rm -rf {} +
       find . -name "build" -exec rm -rf {} +
       find . -name "build_release" -exec rm -rf {} +
       find . -name "*.pbstream" -exec rm {} +
       find . -name "*.pgm" -exec rm {} +
       find . -name "*.samples.json" -exec rm {} +
       popd
    fi
    
    pushd docker/people
    docker build -f Dockerfile.jetson-prod -t cabot_people-jetson:$tag .
    docker image tag cabot_people-jetson:$tag cabot_people-jetson-prod:latest
    docker image tag cabot_people-jetson:$tag cmucal/cabot_people-jetson:$tag
    popd
fi
