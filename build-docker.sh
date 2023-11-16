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
    echo "Usage: $0 <option> [<service>, ...]"
    echo ""
    echo "example $0 -p -i -w     # build all"
    echo ""
	echo "-p          prebuild images"
	echo "-i          build images with docker-compoose files"
	echo "-w          build workspaces"
	echo ""
	echo "Available services:"
	declare -A services_dict
    for dcfile in ${dcfiles[@]}; do
		services=$(docker compose -f $dcfile config --services 2> /dev/null)
		for service in ${services[@]}; do
			if [[ ! -v services_dict[$service] ]]; then
				echo "  $service"
			fi
			services_dict[$service]=1
		done
	done
}

pwd=$(pwd)
scriptdir=$(dirname $0)
cd $scriptdir
scriptdir=$(pwd)
prefix=$(basename $scriptdir)
prefix_pb=${prefix}_

arch=$(uname -m)
time_zone=$(cat /etc/timezone)

prebuild=0
build_img=0
build_ws=0

while getopts "hpiw" arg; do
    case $arg in
	h)
	    help
	    exit
	    ;;
	p)
	    prebuild=1
	    ;;
	i)
	    build_img=1
	    ;;
	w)
	    build_ws=1
	    ;;
    esac
done
shift $((OPTIND-1))
targets=$@

if [[ ! -z $targets ]]; then
	# make target dict
	declare -A target_dict
	for target in ${targets[@]}; do
		target_dict[$target]=1
	done
fi

if [[ $prebuild -eq 1 ]]; then
    blue "Building prebuild images"
    ./cabot-navigation/build-docker.sh -P ${prefix}
    ./cabot-people/build-docker.sh -P ${prefix}
    # ./cabot-drivers/build-docker.sh -p
fi

dcfiles=$(ls docker-compose* | grep -v jetson | grep -v vs)

if [[ $build_img -eq 1 ]]; then
    blue "Building images"
    for dcfile in ${dcfiles[@]}; do
		blue "Building $dcfile"
		services=$(docker compose -f $dcfile config --services)
		if [[ $? -ne 0 ]]; then
			exit
		fi
		for service in ${services[@]}; do
			# check if target_dict exists and service is in the target_dict
			if declare -p target_dict &> /dev/null && [[ ! -v target_dict[$service] ]]; then
				continue
			fi
			blue "Building image of $dcfile, $service"
			docker compose -f $dcfile build \
				--build-arg PREFIX=${prefix} \
				--build-arg UID=$UID \
				--build-arg TZ=$time_zone \
				$service
			if [[ $? -ne 0 ]]; then
				exit
			fi
		done
    done
fi

declare -A built

if [[ $build_ws -eq 1 ]]; then
    blue "Building workspaces"
    for dcfile in ${dcfiles[@]}; do
		blue "Building $dcfile"
		services=$(docker compose -f $dcfile config --services)
		if [[ $? -ne 0 ]]; then
			exit
		fi
		for service in ${services[@]}; do
			# check if target_dict exists and service is in the target_dict
			if declare -p target_dict &> /dev/null && [[ ! -v target_dict[$service] ]]; then
				continue
			fi
			blue "Building workspace of $dcfile, $service"

			# check if volume src dir is already built or not
			dirs=$(docker compose -f $dcfile config $service | grep target | grep src | cut -d: -f2)
			flag=true
			for dir in ${dirs[@]}; do
				if [[ ! -v built[$dir] ]]; then
					flag=false
				fi
				built[$dir]=1
			done
			if $flag; then
				red "already built"
				continue
			fi
			docker compose -f $dcfile run --rm $service /launch.sh build
			if [[ $? -ne 0 ]]; then
			exit
			fi
		done
    done
fi

if [[ $prebuild -eq 0 ]] && [[ $build_img -eq 0 ]] && [[ $build_ws -eq 0 ]]; then
	help
fi	