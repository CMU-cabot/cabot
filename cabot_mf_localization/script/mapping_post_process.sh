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

robot=$CABOT_MODEL
WORKDIR=/home/developer/post_process
QUIT_WHEN_ROSBAG_FINISH=${QUIT_WHEN_ROSBAG_FINISH:-true}
PLAYBAG_RATE_CARTOGRAPHER=${PLAYBAG_RATE_CARTOGRAPHER:-1.0}
PLAYBAG_RATE_PC2_CONVERT=${PLAYBAG_RATE_PC2_CONVERT:-1.0}

# topic
points2_topic='/velodyne_points'
imu_topic='/imu/data'

function blue {
    echo -en "\033[36m"  ## blue
    echo $@
    echo -en "\033[0m"  ## reset color
}

# check if CABOT_MODEL is specified
if [ "$robot" = "" ]; then
    echo "CABOT_MODEL must be specified"
    exit
fi
blue "using CABOT_MODEL=$CABOT_MODEL in post processing"

if [[ ! -e $WORKDIR/${BAG_FILENAME}.carto-converted ]]; then
    ros2 launch mf_localization_mapping convert_rosbag_for_cartographer.launch.py \
	      points2:=${points2_topic} \
	      imu:=${imu_topic} \
	      rate:=${PLAYBAG_RATE_PC2_CONVERT} \
	      convert_points:=true \
	      bag_filename:=$WORKDIR/${BAG_FILENAME}
else
    blue "skipping $WORKDIR/${BAG_FILENAME}.carto-converted"
fi

if [[ ! -e $WORKDIR/${BAG_FILENAME}.carto-converted.loc.samples.json ]] || [[ ! -e $WORKDIR/${BAG_FILENAME}.carto-converted.pbstream ]]; then
    com="ros2 launch mf_localization_mapping demo_2d_VLP16.launch.py \
	      save_samples:=true \
	      save_state:=true \
	      points2:=${points2_topic} \
	      imu:=${imu_topic} \
	      robot:=${robot} \
	      delay:=10 \
	      rate:=${PLAYBAG_RATE_CARTOGRAPHER} \
	      quit_when_rosbag_finish:=${QUIT_WHEN_ROSBAG_FINISH} \
	      bag_filename:=$WORKDIR/${BAG_FILENAME}.carto-converted"
    echo $com
    eval $com
else
    blue "skipping $WORKDIR/${BAG_FILENAME}.carto-converted.loc.samples.json"
    blue "skipping $WORKDIR/${BAG_FILENAME}.carto-converted.pbstream"
fi


if [[ ! -e $WORKDIR/${BAG_FILENAME}.carto-converted.pgm ]]; then
    ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
	   -pbstream_filename $WORKDIR/${BAG_FILENAME}.carto-converted.pbstream \
	   -map_filestem $WORKDIR/${BAG_FILENAME}.carto-converted
else
    blue "skipping $WORKDIR/${BAG_FILENAME}.carto-converted.pgm"
fi

# convert pgm to png
if [[ ! -e $WORKDIR/${BAG_FILENAME}.carto-converted.png ]]; then
    convert $WORKDIR/${BAG_FILENAME}.carto-converted.pgm $WORKDIR/${BAG_FILENAME}.carto-converted.png
else
    blue "skipping $WORKDIR/${BAG_FILENAME}.carto-converted.png"
fi

# extract floor map info from the ros map files
if [[ ! -e $WORKDIR/${BAG_FILENAME}.carto-converted.info.txt ]]; then
    # origin_x, origin_y, ppm
    ros2 run mf_localization_mapping extract_floormap_info_from_yaml.py \
        --input $WORKDIR/${BAG_FILENAME}.carto-converted.yaml \
        --output $WORKDIR/${BAG_FILENAME}.carto-converted.info.txt
    # width, height
    identify -format "width: %w\nheight: %h\n" $WORKDIR/${BAG_FILENAME}.carto-converted.pgm \
        | tee -a $WORKDIR/${BAG_FILENAME}.carto-converted.info.txt
else
    blue "skipping $WORKDIR/${BAG_FILENAME}.carto-converted.info.txt"
fi