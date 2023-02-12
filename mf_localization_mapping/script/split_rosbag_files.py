#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (c) 2021  IBM Corporation
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
import yaml
import rosbag
import os


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--input", required=True)
    parser.add_argument("-o", "--output_dir")
    parser.add_argument("-n", "--n_split", default=10, type=float)
    args = parser.parse_args()

    input_bag = args.input
    output_dir = args.output_dir
    n_split = args.n_split

    with rosbag.Bag(input_bag) as bag:
        info_dict = yaml.load(bag._get_yaml_info())
        start = info_dict["start"]
        end = info_dict["end"]
        duration = info_dict["duration"]
        dt = duration/n_split

        segment = 0

        start_segment = start
        end_segment = start + dt

        outbag = None
        for topic, msg, t in bag.read_messages():
            if start_segment <= t.to_sec():
                if outbag is None:
                    if output_dir is not None:
                        base_name = os.path.basename(input_bag)
                        file_name = base_name[:-4]+".part"+str(segment)+".bag"
                        output_file = os.path.join(output_dir, file_name)
                        outbag = rosbag.Bag(output_file, 'w')

            if outbag is not None:
                outbag.write(topic, msg, t)

            if end_segment < t.to_sec():
                start_segment += dt
                end_segment += dt
                segment += 1
                outbag.close()
                outbag = None
