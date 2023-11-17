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

## termination hook
trap ctrl_c INT QUIT TERM

function ctrl_c() {
    exit
}

source ./cabot-common/build-utils.sh

function help {
    echo "Usage: $0 <option>"
    echo "$0 [<option>] [<target>]"
    echo ""
    echo "-h                    show this help"
    echo "-n                    no cache option to build docker image"
    echo "-t <time_zone>        set time zone"
    echo "-u <uid>              replace uid"
    echo "-p                    prebuild images"
    echo "-P <prefix>           prebuild with prefix"
    echo "-i                    build images"
    echo "-w                    build workspace"

	echo "Available services:"
    show_available_services dcfiles
}

pwd=$(pwd)
scriptdir=$(dirname $0)
cd $scriptdir
scriptdir=$(pwd)
prefix=$(basename $scriptdir)
build_dir=$scriptdir/cabot-common/docker

option="--progress=auto"
time_zone=$(cat /etc/timezone)
uid=$UID
prebuild=0
build_image=0
build_workspace=0

export DOCKER_BUILDKIT=1
export COMPOSE_DOCKER_CLI_BUILD=1

while getopts "hnt:u:pP:iw" arg; do
    case $arg in
	h)
	    help
	    exit
	    ;;
	n)
	    option="$option --no-cache"
	    ;;
	t)
	    time_zone=$OPTARG
	    ;;
    u)
        uid=$OPTARG
        ;;
    p)
        prebuild=1
        ;;
    P)
        prebuild=1
        prefix=${OPTARG}
        ;;
    i)
        build_image=1
        ;;
	w)
	    build_workspace=1
	    ;;
    esac
done
shift $((OPTIND-1))
targets=$@

if [[ $prebuild -eq 1 ]]; then
	./cabot-navigation/build-docker.sh -P $prefix
    if [ $? -ne 0 ]; then exit 1; fi
    ./cabot-drivers/build-docker.sh -P $prefix
    if [ $? -ne 0 ]; then exit 1; fi
    ./cabot-people/build-docker.sh -P $prefix
    if [ $? -ne 0 ]; then exit 1; fi
fi

readarray -t dcfiles < <(ls docker-compose* | grep -v jetson | grep -v vs)

if [[ $build_image -eq 1 ]]; then
    build_image dcfiles targets option time_zone uid prefix
    if [ $? != 0 ]; then exit 1; fi
fi

if [[ $build_workspace -eq 1 ]]; then
    build_workspace dcfiles targets
    if [ $? != 0 ]; then exit 1; fi
fi

if [[ $prebuild -eq 0 ]] && [[ $build_image -eq 0 ]] && [[ $build_workspace -eq 0 ]]; then
	help
fi