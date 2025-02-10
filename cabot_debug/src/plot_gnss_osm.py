#!/usr/bin/env python3

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

import argparse
import io
import os
from pathlib import Path

import folium
import numpy as np
from PIL import Image
from rosbags.highlevel import AnyReader

STATUS_NO_FIX = -1
STATUS_FIX = 0
STATUS_SBAS_FIX = 1
STATUS_GBAS_FIX = 2


def main():
    parser = argparse.ArgumentParser("Tool to plot GNSS fix topic on OpenStreetMap")
    parser.add_argument("-i", "--input_bag", required=True, help="input rosbag file")
    parser.add_argument("-o", "--output_file", default="map.html")
    parser.add_argument("-t", "--threshold", default=0.1, type=float, help="stdev threshold to classify RTK float estimation to accurate or inaccurate")
    parser.add_argument("-r", "--radius_min", default=0.1, type=float, help="minumum point radius")
    parser.add_argument("-a", "--animation", default=False, action="store_true")
    parser.add_argument("-d", "--output_dir", default="images", help="directory to animation output image files")
    parser.add_argument("-p", "--plot_interval", default=0.0, type=float)
    parser.add_argument("-z", "--zoom_start", default=18, help="zoom start")
    parser.add_argument("-Z", "--max_zoom", default=20, help="max zoom")
    parser.add_argument("--fix_topic", default="/ublox/fix")
    parser.add_argument("--lat", "--latitude", default=None, type=float, help="latitude for plot origin")
    parser.add_argument("--lng", "--longitude", default=None, type=float, help="longitude for plot origin")
    parser.add_argument("-v", "--verbose", default=False, action="store_true")
    args = parser.parse_args()

    input_bag = args.input_bag
    output_file = args.output_file
    stdev_threshold = args.threshold
    radius_min = args.radius_min
    export_animation_img = args.animation
    output_dir = args.output_dir
    plot_interval = args.plot_interval
    zoom_start = args.zoom_start
    max_zoom = args.max_zoom
    fix_topic = args.fix_topic
    lat_arg = args.lat
    lng_arg = args.lng
    verbose = args.verbose

    folium_map = None
    origin = None
    period_counter = 0.0

    with AnyReader([Path(input_bag)]) as reader:
        total_count = 0
        status_count = {
            STATUS_NO_FIX: 0,
            STATUS_FIX: 0,
            STATUS_SBAS_FIX: 0,
            STATUS_GBAS_FIX: 0,
        }

        connections = [x for x in reader.connections if x.topic == fix_topic]
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            topic = connection.topic

            if topic == fix_topic:
                msg = reader.deserialize(rawdata, connection.msgtype)
                timestamp_seconds = timestamp*1e-9  # nano seconds -> seconds
                status = msg.status.status
                lat = msg.latitude
                lng = msg.longitude
                cov = msg.position_covariance
                stdev = np.sqrt(cov[0])

                if verbose:
                    print(F"time={timestamp_seconds}, status={status}, lat={lat}, lng={lng}, cov[0]={cov[0]}")

                if origin is None:
                    # use lat lng argument
                    if not (lat_arg is None and lng_arg is None):
                        origin = {
                            "lat": lat_arg,
                            "lng": lng_arg
                        }
                    # do not use invalid lat lng values
                    elif not (lat == 0.0 and lng == 0.0):
                        origin = {
                            "lat": lat,
                            "lng": lng
                        }
                    else:
                        continue

                    folium_map = folium.Map(
                        location=[
                            origin["lat"],
                            origin["lng"]
                        ],
                        zoom_start=zoom_start,
                        max_zoom=max_zoom)

                # set color to maker based on status and stdev
                c = "black"
                if status == STATUS_GBAS_FIX:
                    c = "blue"
                elif status == STATUS_SBAS_FIX:
                    c = "green"
                elif status == STATUS_FIX:
                    if stdev <= stdev_threshold:
                        c = "yellow"
                    else:
                        c = "red"
                elif status == STATUS_NO_FIX:
                    c = "black"
                else:
                    c = "black"

                # update count
                status_count[status] += 1
                total_count += 1

                # plot points at fixed time intervals
                plot_point = False
                if plot_interval == 0.0:
                    plot_point = True
                elif int(timestamp_seconds/plot_interval) != period_counter:
                    period_counter = int(timestamp_seconds/plot_interval)
                    plot_point = True

                if plot_point:
                    # plot points if covariance has a valid value.
                    if 0.0 <= cov[0] and folium_map is not None:
                        radius = stdev if radius_min < stdev else radius_min
                        folium.Circle(
                            location=[lat, lng],
                            radius=radius,
                            color=c,
                            fill=True,
                            opacity=0.5,
                            fill_opacity=0.05
                        ).add_to(folium_map)

                    # save images for creating animation
                    # this feature requires geckodriver
                    if export_animation_img:
                        img_data = folium_map._to_png(1)
                        img = Image.open(io.BytesIO(img_data))
                        img.save(
                            os.path.join(
                                output_dir,
                                'image'+str(total_count-1)+'.png'
                            )
                        )
                        folium_map._png_image = None  # clear rendered png data

        # STATUS_GBAS_FIX rate
        fix_rate = float(status_count[STATUS_GBAS_FIX])/float(total_count)
        if verbose:
            print("GBAS FIX rate = "+str(fix_rate))

    folium_map.save(output_file)


if __name__ == "__main__":
    main()
