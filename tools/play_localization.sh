#!/bin/bash

# Copyright (c) 2025  IBM Corporation
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

function help {
    echo "Usage: $0 [<options>] <bag_dir>"
    echo ""
    echo "All options are forwarded to the container-side play_localization.sh."
    echo "The last positional argument must be the host bag directory."
}

function is_ros2_bag_dir {
    [[ -f $1/metadata.yaml ]]
}

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`
cd $scriptdir/../

if [[ $# -eq 0 ]]; then
    help
    exit 1
fi

args=("$@")

if [[ " ${args[*]} " == *" -h "* || " ${args[*]} " == *" --help "* ]]; then
    help
    echo ""
    echo "Container-side help:"
    cmd=(
        docker compose --progress quiet run --rm
        localization-dev
        /launch.sh play -h
    )
    printf '%q ' "${cmd[@]}"
    echo
    exec "${cmd[@]}"
fi

bag_index=-1
for i in "${!args[@]}"; do
    if [[ ${args[$i]} != -* ]]; then
        bag_index=$i
    fi
done

if [[ $bag_index -lt 0 || $bag_index -ne $((${#args[@]} - 1)) ]]; then
    echo "The last positional argument must be the bag directory."
    help
    exit 1
fi

bag=${args[$bag_index]}

if [[ ! -d $bag ]]; then
    echo "Bag directory does not exist: $bag"
    exit 1
fi

if ! is_ros2_bag_dir "$bag"; then
    echo "Bag directory does not look like a ROS 2 bag: $bag"
    echo "Expected metadata.yaml in the bag directory."
    exit 1
fi

bag=$(realpath "$bag")
forwarded_args=("${args[@]:0:$bag_index}" /ros2_topics)

cmd=(
    docker compose run --rm
    -v "$bag:/ros2_topics:ro"
    localization-dev
    /launch.sh play
)

cmd+=("${forwarded_args[@]}")

printf '%q ' "${cmd[@]}"
echo
exec "${cmd[@]}"
