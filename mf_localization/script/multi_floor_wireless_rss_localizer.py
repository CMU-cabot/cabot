#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (c) 2022  IBM Corporation
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

import argparse
import json
import orjson
import yaml

import numpy as np
import matplotlib.pyplot as plt
import rosbag

import geoutil
import resource_utils

from wireless_utils import extract_samples
from wireless_rss_localizer import create_wireless_rss_localizer
from multi_floor_manager import convert_samples_coordinate


def load_all_samples(global_anchor_dict, map_list):
    # global anchor
    global_anchor = geoutil.Anchor(lat = global_anchor_dict["latitude"],
                        lng = global_anchor_dict["longitude"],
                        rotate = global_anchor_dict["rotate"]
                        )

    # load all samples  
    samples_global_all = []
    floor_list = []
    for map_dict in map_list:
        floor = float(map_dict["floor"])
        #floor_str = str(int(map_dict["floor"]))
        #area = int(map_dict["area"]) if "area" in map_dict else 0
        #area_str = str(area)
        #node_id = map_dict["node_id"]
        #frame_id = map_dict["frame_id"]
        anchor = geoutil.Anchor(lat = map_dict["latitude"],
                            lng = map_dict["longitude"],
                            rotate = map_dict["rotate"]
                            )

        samples_filename = resource_utils.get_filename(map_dict["samples_filename"])
        with open(samples_filename, "rb") as f:
            samples = orjson.loads(f.read())

        # convert samples to the coordinate of global_anchor
        samples_global = convert_samples_coordinate(samples, anchor, global_anchor, floor)
        samples_global_all.extend(samples_global)
        floor_list.append(floor)

    floor_list = np.unique(floor_list)

    return samples_global_all, floor_list

def create_floor_localizer(parameter_dict):
    n_neighbors_floor = parameter_dict["n_neighbors_floor"]
    min_beacons_floor = parameter_dict["min_beacons_floor"]
    rssi_offset = parameter_dict["rssi_offset"]
    floor_localizer_class_name = parameter_dict["floor_localizer"]

    ble_floor_localizer = create_wireless_rss_localizer(floor_localizer_class_name, n_neighbors=n_neighbors_floor, min_beacons=min_beacons_floor, rssi_offset=rssi_offset)
    
    return ble_floor_localizer

def main(map_config, bag_file, topics, output, parameter_dict, show_fig, verbose):
    
    # load map config
    with open(map_config) as f:
        map_config_dict = yaml.safe_load(f)

    global_anchor_dict = map_config_dict["anchor"]
    map_list = map_config_dict["map_list"]

    samples_global_all, floor_list = load_all_samples(global_anchor_dict, map_list)

    # create localizer instance
    ble_floor_localizer = create_floor_localizer(parameter_dict)

    n_neighbors_floor = parameter_dict["n_neighbors_floor"]
    min_beacons_floor = parameter_dict["min_beacons_floor"]
    rssi_offset = parameter_dict["rssi_offset"]

    # train
    samples_global_all_ble = extract_samples(samples_global_all, key="iBeacon")
    ble_floor_localizer.fit(samples_global_all_ble)

    if bag_file is None:
        return

    X = []
    floor_raw_all = []
    with rosbag.Bag(bag_file) as bag:
        for topic, msg, t in bag.read_messages(topics):
            
            if "beacons" in topic:
                
                data = json.loads(msg.data)
                beacons = data["data"]
                
                loc = ble_floor_localizer.predict(beacons)
                if loc is None:
                    continue

                floor_raw = np.mean(loc[:,3])
                idx_floor = np.abs(np.array(floor_list) - floor_raw).argmin()
                floor_int = floor_list[idx_floor]

                x = [t.to_sec(), loc[0,0], loc[0,1], loc[0,2], loc[0,3], floor_int] # [timestamp, x, y, z, floor]
                X.append(x) 

                if verbose:
                    print("t="+str(t.to_sec()))
                    print("#beacons="+str(len(beacons)))
                    for b in beacons:
                        print(b)
                    print("x="+str(x))
                    print("----------")

    X = np.array(X)

    if show_fig:
        plt.plot(X[:,0], X[:,4], label="floor_raw") # timestamp, floor
        plt.plot(X[:,0], X[:,5], label="floor") # timestamp, floor_int
        plt.xlabel("timestamp")
        plt.ylabel("floor")
        plt.ylim([np.min(floor_list), np.max(floor_list)])
        plt.legend()
        plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--map_config", required=True)
    parser.add_argument("-b", "--bag", default=None)
    parser.add_argument("-o", "--output", default=None)
    parser.add_argument("--show", default=False, action="store_true", help="show figure")
    parser.add_argument("-v", "--verbose", default=False, action="store_true")
    parser.add_argument("--topics", nargs="*", default=["/wireless/beacons", "/wireless/wifi"])

    # parameters
    parser.add_argument("--floor_localizer", default="SimpleRSSLocalizer")
    parser.add_argument("--rssi_offset", type=float, default=0.0)
    parser.add_argument("--n_neighbors", type=int, default=3)
    parser.add_argument("--min_beacons", type=int, default=1)

    args = parser.parse_args()

    map_config = args.map_config
    bag_file = args.bag
    output = args.output
    show_fig = args.show
    verbose = args.verbose
    topics = args.topics
    
    parameter_dict = {
        "floor_localizer": args.floor_localizer,
        "rssi_offset": args.rssi_offset,
        "n_neighbors_floor": args.n_neighbors,
        "min_beacons_floor": args.min_beacons
    }

    main(map_config, bag_file, topics, output, parameter_dict, show_fig, verbose)
