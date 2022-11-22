#!/bin/bash


bagfile=$1
rate=1

source devel/setup.bash
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

