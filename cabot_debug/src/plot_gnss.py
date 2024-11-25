#!/usr/bin/env python3

# Copyright (c) 2024  Carnegie Mellon University and IBM Corporation
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


from optparse import OptionParser
from collections import defaultdict
import math
import numpy as np
import os
import sys
import rclpy.time
from matplotlib import pyplot as plt

from cabot_common.rosbag2 import BagReader
from tf_bag import BagTfTransformer


if __name__ == "__main__":
    parser = OptionParser(usage="""
    Plot GNSS data

    Example
    {0} -f <bag file>                        # bag file to plot
    """.format(sys.argv[0]))

    parser.add_option('-f', '--file', type=str, help='bag file to plot')
    parser.add_option('-l', '--latlng', action='store_true', help='plot latitude and longitude')
    parser.add_option('--cov', action='store_true', help='plot position_covariance')
    parser.add_option('--cno', action='store_true', help='plot cno')
    parser.add_option('--elev', action='store_true', help='plot elev')
    parser.add_option('--cno_elev', action='store_true', help='plot cno-elev')
    parser.add_option('--cno_threshold', type=float, default=0.0, help='threshold of cno to plot')
    parser.add_option('--elev_threshold', type=float, default=0.0, help='threshold of elev to plot')
    parser.add_option('-s', '--start', type=float, help='start time from the begining', default=0.0)
    parser.add_option('-d', '--duration', type=float, help='duration from the start time', default=99999999999999)
    parser.add_option('--dir', type=str, help='output directry', default=None)

    (options, args) = parser.parse_args()

    if not options.file:
        parser.print_help()
        sys.exit(0)

    bagfilename = options.file
    reader = BagReader(bagfilename)

    reader.set_filter_by_topics([
        "/ublox/fix",
        "/ublox/navsat",
    ])
    reader.set_filter_by_options(options)  # filter by start and duration

    ublox_fix = []
    sv_data = defaultdict(lambda: defaultdict(list))  # sv_data[key1][key2] = []
    while reader.has_next():
        (topic, msg, t, st) = reader.serialize_next()
        if not topic:
            continue

        if topic == "/ublox/fix":
            # NavSatFix
            ublox_fix.append([st, msg.latitude, msg.longitude, msg.altitude, msg.status.status, *msg.position_covariance])
        elif topic == "/ublox/navsat":
            # NavSAT
            svs = msg.sv
            for sv in svs:
                sv_data[sv.gnss_id][sv.sv_id].append([st, sv.cno, sv.elev])

    ublox_fix = list(zip(*ublox_fix))

    if options.latlng:
        plt.figure(figsize=(10, 10))
        plt.scatter(ublox_fix[2], ublox_fix[1], c=ublox_fix[4], label="GNSS")
        plt.xlabel("Longitude [deg]")
        plt.ylabel("Latitude [deg]")
        plt.legend()
        plt.colorbar()
        plt.gca().set_aspect('equal')
        if options.dir:
            plt.savefig(os.path.join(options.dir, "latlng.png"))
        plt.show()

    if options.cov:
        plt.figure(figsize=(10, 10))
        position_covariance_0 = ublox_fix[5]
        deviation = np.sqrt(position_covariance_0)
        print(deviation)
        plt.scatter(ublox_fix[0], deviation, color='blue', label="sqrt(cov) [m]")
        plt.xlim(0)
        plt.xlabel("Elapsed time [s]")
        plt.ylabel("Square root of position covariance [m]")
        plt.legend()
        if options.dir:
            plt.savefig(os.path.join(options.dir, "sqrt_position_covariance.png"))
        plt.show()

    if options.cno:
        plt.figure(figsize=(10, 10))
        for gnss_id in sv_data:
            for sv_id in sv_data[gnss_id]:
                data = list(zip(*sv_data[gnss_id][sv_id]))
                cno_mean = np.mean(data[1])
                elev_mean = np.mean(data[2])
                if options.cno_threshold <= cno_mean and options.elev_threshold <= elev_mean:
                    plt.plot(data[0], data[1], label=f"{gnss_id}-{sv_id} (μ={cno_mean:.1f})")
        plt.xlim(0)
        plt.xlabel("Elapsed time [s]")
        plt.ylabel("cno [dBHz]")
        plt.legend()
        if options.dir:
            plt.savefig(os.path.join(options.dir, "cno.png"))
        plt.show()

    if options.elev:
        plt.figure(figsize=(10, 10))
        for gnss_id in sv_data:
            for sv_id in sv_data[gnss_id]:
                data = list(zip(*sv_data[gnss_id][sv_id]))
                cno_mean = np.mean(data[1])
                elev_mean = np.mean(data[2])
                if options.cno_threshold <= cno_mean and options.elev_threshold <= elev_mean:
                    plt.plot(data[0], data[2], label=f"{gnss_id}-{sv_id} (μ={elev_mean:.1f})")
        plt.xlim(0)
        plt.xlabel("Elapsed time [s]")
        plt.ylabel("elev [deg]")
        plt.legend()
        if options.dir:
            plt.savefig(os.path.join(options.dir, "elevation.png"))
        plt.show()

    if options.cno_elev:
        plt.figure(figsize=(10, 10))
        markers = ['o', 'v', '^', 's', '*', '+', 'x', ""]
        for i, gnss_id in enumerate(sv_data):
            marker = markers[i]
            for sv_id in sv_data[gnss_id]:
                data = list(zip(*sv_data[gnss_id][sv_id]))
                cno_mean = np.mean(data[1])
                elev_mean = np.mean(data[2])
                if options.cno_threshold <= cno_mean and options.elev_threshold <= elev_mean:
                    plt.scatter(data[2], data[1], label=f"{gnss_id}-{sv_id}", marker=marker)
        plt.xlabel("elev [deg]")
        plt.ylabel("cno [dBHz]")
        plt.legend()
        if options.dir:
            plt.savefig(os.path.join(options.dir, "cno-elevation.png"))
        plt.show()
