#!/bin/bash

# Copyright (c) 2022  Carnegie Mellon University
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

WORKDIR=/home/developer/post_process
QUIT_WHEN_ROSBAG_FINISH=${QUIT_WHEN_ROSBAG_FINISH:-true}

function blue {
    echo -en "\033[36m"  ## blue
    echo $@
    echo -en "\033[0m"  ## reset color
}

if [[ ! -e $WORKDIR/${BAG_FILENAME}.bag.carto-converted.bag ]]; then
    roslaunch mf_localization_mapping convert_rosbag_for_cartographer.launch convert_points:=true bag_filename:=$WORKDIR/${BAG_FILENAME}.bag
else
    blue "skipping $WORKDIR/${BAG_FILENAME}.bag.carto-converted.bag"
fi

if [[ ! -e $WORKDIR/${BAG_FILENAME}.bag.carto-converted.bag.loc.samples.json ]] || [[ ! -e $WORKDIR/${BAG_FILENAME}.bag.carto-converted.bag.pbstream ]]; then
    roslaunch mf_localization_mapping demo_2d_VLP16.launch \
	      save_samples:=true \
	      save_state:=true \
	      delay:=10 \
	      quit_when_rosbag_finish:=${QUIT_WHEN_ROSBAG_FINISH} \
	      bag_filename:=$WORKDIR/${BAG_FILENAME}.bag.carto-converted.bag
else
    blue "skipping $WORKDIR/${BAG_FILENAME}.bag.carto-converted.bag.loc.samples.json"
    blue "skipping $WORKDIR/${BAG_FILENAME}.bag.carto-converted.bag.pbstream"
fi


if [[ ! -e $WORKDIR/${BAG_FILENAME}.bag.carto-converted.pgm ]]; then
    rosrun cartographer_ros cartographer_pbstream_to_ros_map \
	   -pbstream_filename $WORKDIR/${BAG_FILENAME}.bag.carto-converted.bag.pbstream \
	   -map_filestem $WORKDIR/${BAG_FILENAME}.bag.carto-converted
else
    blue "skipping $WORKDIR/${BAG_FILENAME}.bag.carto-converted.pgm"
fi
