#!/bin/bash

# Copyright (c) 2020, 2023  Carnegie Mellon University, IBM Corporation, and others
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

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

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`
project=`basename $scriptdir`

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
       cp -r ./cabot_common ./docker/people/src/
       cp -r ./cabot_msgs ./docker/people/src/
       cp -r ./mf_localization_msgs ./docker/people/src/
       cp -r ./cabot_people ./docker/people/src/
       cp -r ./queue_people_py ./docker/people/src/
       cp -r ./track_people_py ./docker/people/src/
       cp -r ./track_people_cpp ./docker/people/src/
       cp -r ./queue_utils_py ./docker/people/src/
       cp -r ./queue_msgs ./docker/people/src/
       cp -r ./docker/prebuild/humble-custom/people/people_msgs ./docker/people/src/
       cp -r ./track_people_msgs ./docker/people/src/track_people_msgs
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
    com="docker build -f Dockerfile.jetson-prod -t cabot-people-jetson:$tag --build-arg FROM_IMAGE=${project}-people-jetson ."
    blue "$com"
    eval $com
    com="docker image tag cabot-people-jetson:$tag cabot-people-jetson-prod:latest"
    blue "$com"
    eval $com
    com="docker image tag cabot-people-jetson:$tag cmucal/cabot_people-jetson:$tag"
    blue "$com"
    eval $com
    popd
fi
