#!/bin/bash


rate=1
test=0

while getopts "t" arg; do
    case $arg in
	t)
	    test=1
	    ;;
    esac
done
shift $((OPTIND-1))

bagfile=$1

source devel/setup.bash

if [[ $test -eq 1 ]]; then
    roslaunch mf_localization_mapping test_multi_floor_localization.launch \
	      convert_points:=false \
	      map_config_file:=/home/developer/loc_ws/src/cabot_sites/cabot_sites_cmu/cabot_site_cmu_3d/maps/cmu_20200310_1f-5f.yaml \
	      scan:=scan \
	      bag_filename:=$bagfile \
	      record_bag:=false \
	      
else
    roslaunch mf_localization_mapping demo_2d_VLP16.launch \
	      save_samples:=false \
	      save_state:=true \
	      delay:=10 \
	      bag_filename:=$bagfile \
	      load_state_filename:=/home/developer/cabot-tbd/tepper-2f_2022-09-30-14-42-54.bag.carto-converted.bag.pbstream \
	      start_trajectory_with_default_topics:=true \
	      scan:=scan \
	      imu:=cabot/imu/data \
	      play_limited_topics:=true \
	      robot:=cabot2-gt1 \
	      configuration_basename:=cartographer_2d_mapping_localization.lua \
	      rate:=$rate
fi
