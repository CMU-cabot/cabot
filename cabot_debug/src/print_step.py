#!/usr/bin/env python3

# Copyright (c) 2020, 2025  Carnegie Mellon University and Miraikan
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


import json
import logging
import os
import re
import subprocess
import sys
from copy import deepcopy
from optparse import OptionParser

import numpy as np
import yaml
from rosidl_runtime_py import message_to_csv
from rosidl_runtime_py import message_to_yaml
from ament_index_python.packages import get_package_share_directory

from cabot_common.rosbag2 import BagReader

logging.basicConfig(level=logging.INFO)

this_file_dir = os.path.dirname(os.path.abspath(__file__))
cabot_ui_dir = os.path.join(this_file_dir, "../../../../../cabot-navigation/cabot_ui")
sys.path.append(cabot_ui_dir)
from cabot_ui import geoutil
from cabot_ui import geojson
# python -m pip install transforms3d # for cabot_ui
# sudo apt-get install ros-galactic-tf-transformations # for transforms3d

parser = OptionParser(usage="""
print memo
""".format(sys.argv[0]))

parser.add_option('-f', '--file', type=str, help='bag file to print')
parser.add_option('-s', '--start', type=float, help='start time from the begining', default=0.0)
parser.add_option('-d', '--duration', type=float, help='duration from the start time', default=99999999999999)
parser.add_option('-t', '--topic', type=str, action='append', default=[], help='topics to be printed')
parser.add_option('-i', '--info', action='store_true', help='print info')
parser.add_option('-1', '--once', action='store_true', help='print only one message')
parser.add_option('-y', '--yaml', action='store_true', help='print message in yaml')
parser.add_option('-g', '--geojson', action='store_true', help='print geojson')

(options, args) = parser.parse_args()

if not options.file:
    parser.print_help()
    sys.exit(0)

options.topic.append("/memo")
if options.geojson:
    options.topic.append("/cabot/pose_log")
    options.topic.append("/current_map_filename")

logging.info(options)
bagfilename = options.file
reader = BagReader(bagfilename)

if options.info:
    for info in sorted(reader.topic_types, key=lambda x: x.name):
        print(f"{info.name:80s}{reader.message_counts[info.name]:10d} {info.type}")

    meta = reader.info
    print(f"Bag Size:   {meta.bag_size / 1024 / 1024:.2f} MB")
    print(f"Start Time: {meta.starting_time}")
    if hasattr(meta.duration, "nanoseconds"):
        print(f"Duration:   {meta.duration} ({meta.duration.nanoseconds/1e9:.2f} seconds)")
    else:   # back compatibility for galactic
        print(f"Duration:   {meta.duration} ({meta.duration.seconds:.2f} seconds)")

    sys.exit(0)

reader.set_filter_by_topics(options.topic)
reader.set_filter_by_options(options)  # filter by start and duration

messages = {}
features = []

def get_geojson(pose_log, hulop_content, heading=0):
    stamp = int(pose_log.header.stamp.sec * 1000 + pose_log.header.stamp.nanosec / 1000000)
    return {
        "type": "Feature",
        "geometry": {
            "type": "Point",
            "coordinates": [
                pose_log.lng, pose_log.lat
            ]
        },
        "properties": {
            "parking": 99,
            "hulop_height": int(pose_log.floor),
            "escalator": 99,
            "hulop_file": "EDITOR",
            "hulop_heading": heading,
            "nursing": 99,
            "brail_tile": 99,
            "lon": pose_log.lng,
            "hulop_content": f"{hulop_content}",
            "hulop_sub_category": "_cabot_memo_",
            "hulop_angle": 60,
            "facil_id": f"EDITOR_facil_{stamp}",
            "toilet": 99,
            "elevator": 99,
            "barrier": 99,
            "hulop_major_category": "_nav_poi_",
            "hulop_minor_category": "_line_, _priority_low_",
            "facil_type": 99,
            "lat": pose_log.lat
        },
        "_id": f"EDITOR_facil_{stamp}",
    }

pose_log_left = None
pose_log_right = None
pose_log_midpoint = None
anchor_rotate = 0.0

def make_geojson_entries(msg):
    global pose_log_left, pose_log_right, pose_log_midpoint, anchor_rotate
    if '/cabot/pose_log' in messages:
        pose_log = messages['/cabot/pose_log']
        if msg.data == "left":
            pose_log_left = deepcopy(pose_log)
        elif msg.data == "right":
            pose_log_right = deepcopy(pose_log)
            pose_log_midpoint = deepcopy(pose_log)
            pose_log_midpoint.lng = (pose_log_left.lng + pose_log_right.lng) / 2
            pose_log_midpoint.lat = (pose_log_left.lat + pose_log_right.lat) / 2
            pose_log_midpoint.header.stamp.sec = int((pose_log_left.header.stamp.sec + pose_log_right.header.stamp.sec) / 2)

            # Convert to Cartesian coordinates
            left = geoutil.Latlng(lat=pose_log_left.lat, lng=pose_log_left.lng)
            right = geoutil.Latlng(lat=pose_log_right.lat, lng=pose_log_right.lng)
            anchor = geoutil.Anchor(lat=pose_log_left.lat, lng=pose_log_left.lng, rotate=anchor_rotate)
            left_xy = geoutil.global2local(left, anchor)
            right_xy = geoutil.global2local(right, anchor)
            angle_rad = np.arctan2(right_xy.y - left_xy.y, right_xy.x - left_xy.x)
            heading = (angle_rad * 180 / np.pi + anchor.rotate) % 360 - 180
            print(f"rotate: {anchor.rotate}")
            print(f"left: {left_xy}")
            print(f"right: {right_xy}")
            print(f"heading: {heading}")
            
            # Find the coordinates of the endpoints of the closest links
            # min_link, min_dist = geojson.Object.get_nearest_link(entry)
            print("----------------")

            entry = get_geojson(pose_log_left, "left", heading)
            features.append(entry)
            entry = get_geojson(pose_log_right, "right", heading)
            features.append(entry)
            entry = get_geojson(pose_log_midpoint, "step", heading)
            features.append(entry)


while reader.has_next():
    try:
        (topic, msg, t, st) = reader.serialize_next()
    except:
        continue
    if not topic:
        continue
    
    # get anchor_rotate from map data
    if topic == "/current_map_filename":
        # msg.data example : "package://cabot_site_miraikan_3d/maps/miraikan_outdoor_north_mapping_2024-09-04-16-15-45.yaml"
        match = re.search(r'package://([^/]+)/', msg.data)
        if match:
            site_package_name = match.group(1)
        else:
            print(f"site package name not found from {msg.data}.")
            sys.exit(1)
        package_share_directory = get_package_share_directory(site_package_name)

        if package_share_directory is not None:
            sitedir = get_package_share_directory(site_package_name)
            config_path = os.path.join(sitedir, "config", "config.sh")
        else:
            print(f"package share directory not found for {site_package_name}.")
            print(f"Clone {site_package_name} into cabot-navigation/cabot_sites and run build-workspace.sh -o.")
        
        try:
            map_value = subprocess.check_output(
                ["bash", "-c", f"(sitedir='{sitedir}';gazebo=0;source {config_path};echo $map)"],
                universal_newlines=True,
            ).strip()
        except subprocess.CalledProcessError:
            map_value = ""
        if not map_value:
            print(f"Please check config/config.sh in site package ({sitedir}) to set map and world")
            sys.exit(1)
            
        with open(map_value, 'r') as file:
            data = yaml.safe_load(file)

        anchor_rotate = data['anchor']['rotate']
        print(f"Anchor rotate: {anchor_rotate}")

    # make step poi
    if topic == "/memo":
        if options.geojson:
            make_geojson_entries(msg)
        elif options.yaml:
            print("/memo")
            print(f"{message_to_yaml(msg)}")
            for key, value in messages.items():
                print(f"{key}")
                print(f"{message_to_yaml(value)}")
        else:
            print(f"/memo,{msg.data}")
            print(f"{key},{message_to_csv(value)}")
    else:
        messages[topic] = msg

    if options.once:
        break


if options.geojson:
    print(json.dumps({
        "type": "FeatureCollection",
        "features": features
    }, indent=4))
