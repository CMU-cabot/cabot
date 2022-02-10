#!/bin/bash

# Copyright (c) 2022  Carnegie Mellon University, IBM Corporation, and others
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


function join_by {
  local d=${1-} f=${2-}
  if shift 2; then
    printf %s "$f" "${@/#/$d}"
  fi
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
    echo "Usage: $0 <option>"
    echo "Example usage"
    echo "$0 -t latest -i all -a tag -o <registory>        # to tag all built images for a registory"
    echo "$0 -t latest -i all -a push -o <registory>       # to push all tagged images to the registory"
    echo ""
    echo "-h                 show this help"
    echo "-a <action>        $(join_by '|' $all_actions)"
    echo "   tag : docker tag  <prefix>_<image>         <registry>/cabot_<image>:<tag>"
    echo "   pull: docker pull <registry>/cabot_<image>:<tag>"
    echo "         docker tag  <registry>/cabot_<image>:<tag> <prefix>_<image>"
    echo "   push: docker push <registry>/cabot_<image>:<tag>"
    echo "   rmi : docker rmi  <prefix>_<image>"
    echo "   list: list available tags for <registry>/cabot_<image>"
    echo "   del : TBD delete <registory>/cabot_<image>:<tag>"
    echo "   tz  : overwrite image timezone with the host timezone if needed"
    echo ""
    echo "-i <image name>    $(join_by '|' $all_images)"
    echo "-o <registry>      dockerhub organization or private server"
    echo "-t <tag name>      tagname (default=latest)"
    echo "-n                 do not overwrite timezone when pulling image"
    echo "-z <time zone>     specify time zone (default=$(cat /etc/timezone) - /etc/timezone)"
}

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

all_actions="tag pull push list rmi del tz"
all_images="ros1 ros2 bridge localization people people-nuc ble_scan"

option="--progress=tty"
debug=0
pwd=`pwd`
prefix_option=
prefix=`basename $pwd`
tagname=latest
images=
action=
org=
no_tz_overwrite=0
local_tz=$(cat /etc/timezone)

while getopts "ht:i:a:o:r:nz:" arg; do
    case $arg in
	h)
	    help
	    exit
	    ;;
	t)
	    tagname=$OPTARG
	    ;;
	i)
	    images=$OPTARG
	    ;;
	a)
	    action=$OPTARG
	    ;;
	o)
	    org=$OPTARG
	    ;;
	n)
	    no_tz_overwrite=1
	    ;;
	z)
	    local_tz=$OPTARG
	    ;;
    esac
done
shift $((OPTIND-1))

error=0
pat=$(join_by "|" $all_actions)
if [ -z $action ] || [[ ! $action =~ ^($pat)$ ]]; then
    red "need to specify action $pat"
    error=1
fi

pat="all|"$(join_by "|" $all_images)
if [ -z $images ] || [[ ! $images =~ ^($pat)$ ]]; then
    red "need to specify image, $pat"
    error=1
fi

if [ -z $tagname ]; then
    red "need to specify a tag on repository"
    error=1
fi

if [ -z $org ]; then
    red "need to specify dockerhub organization or private repo server address"
    error=1
fi

if [ $error -eq 1 ]; then
    exit
fi

if [ $images == "all" ]; then
    images=$all_images
fi

for image in $images; do
    if [ $action == "tag" ]; then
	com="docker ${action} ${prefix}_${image}:latest ${org}/cabot_${image}:${tagname}"
	echo $com
	eval $com
    fi

    if [ $action == "push" ]; then
	com="docker ${action} ${org}/cabot_${image}:${tagname}"
	echo $com
	eval $com
    fi

    if [ $action == "pull" ]; then
	com="docker ${action} ${org}/cabot_${image}:${tagname}"
	echo $com
	eval $com

	if [ $no_tz_overwrite -eq 0 ]; then
	    image_tz=$(docker run --rm ${org}/cabot_${image}:${tagname} cat /etc/timezone)
	    blue "Image TZ:'$image_tz'    Local TZ:'$local_tz'   - ${prefix}_${image}"
	    if [ "$local_tz" != "$image_tz" ]; then
		blue "Overwrite timezone of $image from $image_tz to $local_tz"
		docker build --build-arg TZ_OVERWRITE=$local_tz --build-arg FROM_IMAGE=${org}/cabot_${image}:${tagname} $scriptdir/docker/timezone -t ${prefix}_${image}
	    else
		blue "Use dockerhub image"
		docker tag ${org}/cabot_${image}:${tagname} ${prefix}_${image}
	    fi
	fi

	if [ $image == "localization" ]; then
	    com="docker tag ${prefix}_localization:latest ${prefix}_topic_checker:latest"
	    echo $com
	    eval $com
	fi

	if [ $image == "ble_scan" ]; then
	    com="docker tag ${prefix}_ble_scan:latest ${prefix}_wifi_scan:latest"
	    echo $com
	    eval $com
	fi
	if [ $image == "people" ]; then
	    for i in 1 2 3; do
		com="docker tag ${prefix}_people ${prefix}_people-rs$i:latest"
		echo $com
		eval $com
	    done
	fi
    fi

    if [ $action == "list" ]; then
	repo="${org}/cabot_${image}"
	
	blue "--Available-Images---------------"
	curl -L -s "https://registry.hub.docker.com/v2/repositories/${repo}/tags?page_size=100" | \
	    jq -r "[\"Last Modified Time      \",\"Image Name:Tag\"], [\"---------------------------\",\"---------------------------------\"], \
            (.[\"results\"][] |	     [.last_updated, @text \"${repo}:\(.name)\"]) | @tsv"
	
	echo ""
	blue "--Rate-Limit---------------------"
	TOKEN=$(curl -s "https://auth.docker.io/token?service=registry.docker.io&scope=repository:${repo}:pull" | jq -r .token)
	curl -s --head -H "Authorization: Bearer $TOKEN" https://registry-1.docker.io/v2/${repo}/manifests/latest | grep -E "^ratelimit" | sed "s/;.*//" 
	SEC=$(curl -s --head -H "Authorization: Bearer $TOKEN" https://registry-1.docker.io/v2/${repo}/manifests/latest | grep -E "^ratelimit-limit" | sed "s/.*;w=//" | sed "s/\r//" )
	echo "in" $(echo "$SEC/3600" | bc) "hours"
    fi

    if [ $action == "rmi" ]; then
	com="docker ${action} ${org}/cabot_${image}:${tagname}"
	echo $com
	eval $com
    fi

    if [ $action == "tz" ]; then
	image_tz=$(docker run --rm ${org}/cabot_${image}:${tagname} cat /etc/timezone)
	blue "Image TZ:'$image_tz'    Local TZ:'$local_tz'   - ${prefix}_${image}"
	if [ "$local_tz" != "$image_tz" ]; then
	    blue "Overwrite timezone of $image from $image_tz to $local_tz"
	    docker build --build-arg TZ_OVERWRITE=$local_tz --build-arg FROM_IMAGE=${org}/cabot_${image}:${tagname} $scriptdir/docker/timezone -t ${prefix}_${image}
	else
	    blue "Use dockerhub image"
	    docker tag ${org}/cabot_${image}:${tagname} ${prefix}_${image}
	fi
    fi
done
